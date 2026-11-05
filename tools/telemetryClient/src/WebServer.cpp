#include "WebServer.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <algorithm>

namespace telemetry {

WebServer::WebServer(int port, const std::string& web_root)
    : m_port(port), m_web_root(web_root) {}

WebServer::~WebServer() {
    stop();
}

bool WebServer::start() {
    if (m_running.load()) return true;

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        std::cerr << "[WebServer] Erreur de création du socket HTTP." << std::endl;
        return false;
    }

    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(SO_REUSEPORT)
    setsockopt(sfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(m_port);

    if (bind(sfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "[WebServer] Échec du bind sur le port web " << m_port << "." << std::endl;
        close(sfd);
        return false;
    }

    if (listen(sfd, 16) < 0) {
        std::cerr << "[WebServer] Échec du listen sur le port web " << m_port << "." << std::endl;
        close(sfd);
        return false;
    }

    m_server_fd = sfd;
    m_running.store(true);
    m_thread = std::thread(&WebServer::server_loop, this);

    std::cout << "[WebServer] Dashboard Web accessible sur http://localhost:" << m_port << std::endl;
    return true;
}

void WebServer::stop() {
    if (!m_running.load()) return;
    m_running.store(false);

    if (m_server_fd >= 0) {
        close(m_server_fd);
        m_server_fd = -1;
    }

    {
        std::lock_guard<std::mutex> lock(m_sse_mutex);
        for (int cfd : m_sse_clients) {
            close(cfd);
        }
        m_sse_clients.clear();
    }

    if (m_thread.joinable()) {
        m_thread.join();
    }
    std::cout << "[WebServer] Serveur Web arrêté." << std::endl;
}

void WebServer::server_loop() {
    while (m_running.load()) {
        struct pollfd pfd{};
        pfd.fd = m_server_fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, 200);
        if (ret <= 0 || !(pfd.revents & POLLIN)) {
            continue;
        }

        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(m_server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            continue;
        }

        // Désactiver l'algorithme de Nagle pour un transfert immédiat à latence zéro
        int nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        // Rendre non-bloquant ou timeout
        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

        char buf[4096];
        ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) {
            close(client_fd);
            continue;
        }
        buf[n] = '\0';

        std::string request_str(buf);
        handle_http_request(client_fd, request_str);
    }
}

void WebServer::handle_http_request(int client_fd, const std::string& request) {
    std::istringstream req_stream(request);
    std::string method, path, version;
    req_stream >> method >> path >> version;

    if (method == "OPTIONS") {
        send_response(client_fd, 204, "text/plain", "");
        close(client_fd);
        return;
    }

    if (method == "GET") {
        if (path == "/" || path == "/index.html") {
            serve_file(client_fd, m_web_root + "/index.html", "text/html; charset=utf-8");
            close(client_fd);
        } else if (path == "/style.css") {
            serve_file(client_fd, m_web_root + "/style.css", "text/css; charset=utf-8");
            close(client_fd);
        } else if (path == "/app.js") {
            serve_file(client_fd, m_web_root + "/app.js", "application/javascript; charset=utf-8");
            close(client_fd);
        } else if (path == "/api/status") {
            std::string st;
            {
                std::lock_guard<std::mutex> lock(m_last_state_mutex);
                st = m_last_status_json;
            }
            send_response(client_fd, 200, "application/json", st);
            close(client_fd);
        } else if (path == "/api/telemetry") {
            std::string telem;
            {
                std::lock_guard<std::mutex> lock(m_last_state_mutex);
                telem = m_last_telemetry_json;
            }
            send_response(client_fd, 200, "application/json", telem);
            close(client_fd);
        } else if (path == "/api/export_csv" || path == "/api/download_csv") {
            serve_csv_download(client_fd);
            close(client_fd);
        } else if (path == "/api/stream") {
            // Configuration du flux Server-Sent Events (SSE) direct sans mise en tampon
            std::string sse_hdr = 
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/event-stream; charset=utf-8\r\n"
                "Cache-Control: no-cache, no-transform\r\n"
                "Connection: keep-alive\r\n"
                "X-Accel-Buffering: no\r\n"
                "Access-Control-Allow-Origin: *\r\n\r\n";

            int send_flags = 0;
#if defined(MSG_NOSIGNAL)
            send_flags |= MSG_NOSIGNAL;
#endif
            send(client_fd, sse_hdr.data(), sse_hdr.size(), send_flags);

            // Envoi immédiat du dernier état
            std::string init_data;
            {
                std::lock_guard<std::mutex> lock(m_last_state_mutex);
                init_data = "event: status\ndata: " + m_last_status_json + "\n\n"
                          + "event: telemetry\ndata: " + m_last_telemetry_json + "\n\n";
            }
            send(client_fd, init_data.data(), init_data.size(), send_flags);

            // Rendre le socket SSE strictement NON-BLOQUANT et TCP_NODELAY
            int nd = 1;
            setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));

            int flags = fcntl(client_fd, F_GETFL, 0);
            if (flags >= 0) {
                fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
            }

            // Conserver le socket ouvert dans la liste SSE
            {
                std::lock_guard<std::mutex> lock(m_sse_mutex);
                m_sse_clients.push_back(client_fd);
            }
            // Ne pas fermer le socket client_fd ici !
        } else {
            send_response(client_fd, 404, "text/plain", "404 Not Found");
            close(client_fd);
        }
    } else if (method == "POST" && path == "/api/command") {
        // Extraction du corps de la requête
        size_t body_pos = request.find("\r\n\r\n");
        std::string body;
        if (body_pos != std::string::npos) {
            body = request.substr(body_pos + 4);
        }

        // Vérification de Content-Length pour s'assurer que le corps est complet
        size_t cl_pos = request.find("Content-Length:");
        if (cl_pos == std::string::npos) {
            cl_pos = request.find("content-length:");
        }
        if (cl_pos != std::string::npos) {
            size_t cl_end = request.find("\r\n", cl_pos);
            if (cl_end != std::string::npos) {
                std::string cl_str = request.substr(cl_pos + 15, cl_end - (cl_pos + 15));
                try {
                    size_t expected_len = std::stoul(cl_str);
                    while (body.size() < expected_len) {
                        char extra[512];
                        ssize_t ex = recv(client_fd, extra, sizeof(extra), 0);
                        if (ex <= 0) break;
                        body.append(extra, ex);
                    }
                } catch (...) {}
            }
        }

        // Nettoyage éventuel (suppression retours à la ligne)
        body.erase(std::remove(body.begin(), body.end(), '\r'), body.end());
        body.erase(std::remove(body.begin(), body.end(), '\n'), body.end());

        // Support d'un format JSON {"service":"FSM","key":"LINE_KP","value":0.58}
        // ou {"service":"FSM","action":"start"} ou {"command":"..."} ou texte brut
        std::string command_to_execute = body;

        auto extract_json_field = [](const std::string& json, const std::string& field) -> std::string {
            std::string token = "\"" + field + "\"";
            size_t pos = json.find(token);
            if (pos == std::string::npos) return "";
            size_t colon = json.find(':', pos + token.size());
            if (colon == std::string::npos) return "";
            size_t start = json.find_first_not_of(" \t\r\n", colon + 1);
            if (start == std::string::npos) return "";

            if (json[start] == '"') {
                size_t end = json.find('"', start + 1);
                if (end != std::string::npos) {
                    return json.substr(start + 1, end - start - 1);
                }
            } else {
                size_t end = json.find_first_of(",}\r\n ", start);
                if (end != std::string::npos) {
                    return json.substr(start, end - start);
                } else {
                    return json.substr(start);
                }
            }
            return "";
        };

        std::string json_srv = extract_json_field(body, "service");
        std::string json_key = extract_json_field(body, "key");
        std::string json_val = extract_json_field(body, "value");
        std::string json_act = extract_json_field(body, "action");
        std::string json_cmd = extract_json_field(body, "command");

        if (!json_key.empty() && !json_val.empty()) {
            std::string srv_prefix = json_srv.empty() ? "FSM" : json_srv;
            command_to_execute = srv_prefix + "." + json_key + "=" + json_val;
        } else if (!json_act.empty()) {
            if (json_act == "start" || json_act == "arm") {
                command_to_execute = "trigger_start";
            } else if (json_act == "stop" || json_act == "emergency") {
                command_to_execute = "emergency_stop";
            } else {
                command_to_execute = json_act;
            }
        } else if (!json_cmd.empty()) {
            command_to_execute = json_cmd;
        }

        if (m_command_cb && !command_to_execute.empty()) {
            m_command_cb(command_to_execute);
        }

        std::string resp_json = "{\"status\":\"ok\",\"command\":\"" + command_to_execute + "\"}";
        send_response(client_fd, 200, "application/json", resp_json);
        close(client_fd);
    } else {
        send_response(client_fd, 405, "text/plain", "Method Not Allowed");
        close(client_fd);
    }
}

void WebServer::serve_file(int client_fd, const std::string& filepath, const std::string& content_type) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        send_response(client_fd, 404, "text/plain", "File Not Found: " + filepath);
        return;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    std::string content = ss.str();

    send_response(client_fd, 200, content_type, content);
}

void WebServer::serve_csv_download(int client_fd) {
    std::string path;
    {
        std::lock_guard<std::mutex> lock(m_csv_mutex);
        path = m_csv_filepath;
    }

    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        send_response(client_fd, 404, "text/plain", "Fichier CSV introuvable ou non encore genere.");
        return;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    std::string content = ss.str();

    std::ostringstream resp_stream;
    resp_stream << "HTTP/1.1 200 OK\r\n";
    resp_stream << "Content-Type: text/csv; charset=utf-8\r\n";
    resp_stream << "Content-Disposition: attachment; filename=\"telemetry_data.csv\"\r\n";
    resp_stream << "Content-Length: " << content.size() << "\r\n";
    resp_stream << "Access-Control-Allow-Origin: *\r\n";
    resp_stream << "Connection: close\r\n\r\n";
    resp_stream << content;

    std::string resp = resp_stream.str();
    int flags = 0;
#if defined(MSG_NOSIGNAL)
    flags |= MSG_NOSIGNAL;
#endif
    send(client_fd, resp.data(), resp.size(), flags);
}

void WebServer::send_response(int client_fd, int status_code, const std::string& content_type, const std::string& body) {
    std::ostringstream ss;
    ss << "HTTP/1.1 " << status_code << " " << (status_code == 200 ? "OK" : (status_code == 204 ? "No Content" : "Error")) << "\r\n";
    ss << "Content-Type: " << content_type << "\r\n";
    ss << "Content-Length: " << body.size() << "\r\n";
    ss << "Access-Control-Allow-Origin: *\r\n";
    ss << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    ss << "Access-Control-Allow-Headers: Content-Type\r\n";
    ss << "Connection: close\r\n\r\n";
    ss << body;

    std::string resp = ss.str();
    int flags = 0;
#if defined(MSG_NOSIGNAL)
    flags |= MSG_NOSIGNAL;
#endif
    send(client_fd, resp.data(), resp.size(), flags);
}

enum class SseStatus {
    Delivered,
    Congested, // Buffer plein mais 0 octet envoyé : client conservé, trame sautée sans bloquer le RT
    Disconnected
};

static SseStatus send_sse_message(int cfd, const std::string& message) {
    size_t total = 0;
    const char* ptr = message.data();
    size_t len = message.size();

    while (total < len) {
        int flags = 0;
#if defined(MSG_NOSIGNAL)
        flags |= MSG_NOSIGNAL;
#endif
#if defined(MSG_DONTWAIT)
        flags |= MSG_DONTWAIT;
#endif
        ssize_t sent = send(cfd, ptr + total, len - total, flags);
        if (sent > 0) {
            total += static_cast<size_t>(sent);
        } else if (sent < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Buffer plein : si aucun octet n'est encore parti, abandonner proprement la trame pour ce client
                if (total == 0) {
                    return SseStatus::Congested;
                }
                // Si une partie a déjà été écrite, le message SSE est corrompu : on déconnecte le client
                return SseStatus::Disconnected;
            }
            return SseStatus::Disconnected;
        } else {
            return SseStatus::Disconnected;
        }
    }
    return SseStatus::Delivered;
}

void WebServer::broadcast_telemetry(const std::string& json_data) {
    {
        std::lock_guard<std::mutex> lock(m_last_state_mutex);
        m_last_telemetry_json = json_data;
    }

    std::string message = "event: telemetry\ndata: " + json_data + "\n\n";

    std::lock_guard<std::mutex> lock(m_sse_mutex);
    std::vector<int> active_clients;

    for (int cfd : m_sse_clients) {
        SseStatus status = send_sse_message(cfd, message);
        if (status == SseStatus::Delivered || status == SseStatus::Congested) {
            active_clients.push_back(cfd);
        } else {
            close(cfd);
        }
    }

    m_sse_clients = std::move(active_clients);
}

void WebServer::broadcast_status(bool robot_connected, const std::string& robot_ip, int robot_port) {
    std::ostringstream ss;
    ss << "{"
       << "\"connected\":" << (robot_connected ? "true" : "false") << ","
       << "\"ip\":\"" << robot_ip << "\","
       << "\"port\":" << robot_port
       << "}";

    std::string json = ss.str();
    {
        std::lock_guard<std::mutex> lock(m_last_state_mutex);
        m_last_status_json = json;
    }

    std::string message = "event: status\ndata: " + json + "\n\n";

    std::lock_guard<std::mutex> lock(m_sse_mutex);
    std::vector<int> active_clients;

    for (int cfd : m_sse_clients) {
        SseStatus status = send_sse_message(cfd, message);
        if (status == SseStatus::Delivered || status == SseStatus::Congested) {
            active_clients.push_back(cfd);
        } else {
            close(cfd);
        }
    }

    m_sse_clients = std::move(active_clients);
}

} // namespace telemetry
