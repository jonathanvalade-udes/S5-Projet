#include "service_ipc_socket.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>

namespace shared::services::ipc {

ServiceIPCSocketServer::~ServiceIPCSocketServer() {
    stop();
}

void ServiceIPCSocketServer::stop() {
    if (m_server_fd >= 0) {
        close(m_server_fd);
        m_server_fd = -1;
    }
    if (!m_socket_path.empty()) {
        unlink(m_socket_path.c_str());
    }
}

bool ServiceIPCSocketServer::init(const std::string& socket_path, shared::commands::ServiceID service_id, ConfigCallback callback) {
    m_socket_path = socket_path;
    m_service_id = service_id;
    m_config_callback = std::move(callback);

    stop();

    m_server_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (m_server_fd < 0) {
        std::cerr << "[IPCSocketServer] Failed to create socket: " << std::strerror(errno) << std::endl;
        return false;
    }

    int flags = fcntl(m_server_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);
    }

    ::unlink(m_socket_path.c_str());

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (m_socket_path.size() >= sizeof(addr.sun_path)) {
        std::cerr << "[IPCSocketServer] Socket path exceeds buffer size: " << m_socket_path << std::endl;
        stop();
        return false;
    }
    std::strncpy(addr.sun_path, m_socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (bind(m_server_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[IPCSocketServer] Failed to bind to " << m_socket_path << ": " << std::strerror(errno) << std::endl;
        stop();
        return false;
    }

    if (listen(m_server_fd, 5) < 0) {
        std::cerr << "[IPCSocketServer] Failed to listen on " << m_socket_path << ": " << std::strerror(errno) << std::endl;
        stop();
        return false;
    }

    chmod(m_socket_path.c_str(), 0666);
    return true;
}

void ServiceIPCSocketServer::process_pending_commands() {
    if (m_server_fd < 0) return;

    struct pollfd pfd{};
    pfd.fd = m_server_fd;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 0);
    if (ret > 0 && (pfd.revents & POLLIN)) {
        int client_fd = accept(m_server_fd, nullptr, nullptr);
        if (client_fd >= 0) {
            handle_client(client_fd);
            close(client_fd);
        }
    }
}

void ServiceIPCSocketServer::handle_client(int client_fd) {
    struct pollfd pfd{};
    pfd.fd = client_fd;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 100);
    if (ret <= 0 || !(pfd.revents & POLLIN)) {
        return;
    }

    shared::headers::PacketHeader hdr{};
    ssize_t n = recv(client_fd, &hdr, sizeof(hdr), MSG_PEEK);
    if (n < static_cast<ssize_t>(sizeof(hdr))) {
        return;
    }

    bool has_crc = (hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t expected_len = sizeof(shared::headers::PacketHeader) + hdr.payloadLength + (has_crc ? sizeof(shared::crc::PacketCRC) : 0);

    std::vector<uint8_t> buffer(expected_len);
    size_t received = 0;
    while (received < expected_len) {
        int p = poll(&pfd, 1, 100);
        if (p <= 0) return;
        ssize_t r = recv(client_fd, buffer.data() + received, expected_len - received, 0);
        if (r <= 0) return;
        received += r;
    }

    shared::headers::PacketHeader rx_hdr{};
    std::vector<uint8_t> rx_payload;
    uint32_t rx_crc = 0;
    if (!shared::headers::deserializePacket(buffer.data(), buffer.size(), rx_hdr, rx_payload, rx_crc)) {
        send_nack(client_fd, hdr.seqNumber, 1);
        return;
    }

    if (rx_hdr.type != shared::headers::HeadersType::Command) {
        send_nack(client_fd, rx_hdr.seqNumber, 2);
        return;
    }

    if (rx_payload.size() < sizeof(shared::commands::CommandsHeader)) {
        send_nack(client_fd, rx_hdr.seqNumber, 3);
        return;
    }

    shared::commands::CommandsHeader cmd_hdr{};
    std::memcpy(&cmd_hdr, rx_payload.data(), sizeof(cmd_hdr));

    bool ok = false;
    if (cmd_hdr.commandID == shared::commands::CommandID::SetConfigKey) {
        constexpr size_t EXPECTED_PAYLOAD = sizeof(shared::commands::CommandsHeader) + sizeof(shared::commands::config::SetConfigPayload);
        if (rx_payload.size() < EXPECTED_PAYLOAD) {
            send_nack(client_fd, rx_hdr.seqNumber, 5);
            return;
        }

        const auto* cfg_payload = reinterpret_cast<const shared::commands::config::SetConfigPayload*>(rx_payload.data() + sizeof(cmd_hdr));
        char key_buf[shared::commands::config::MAX_KEY_LEN + 1];
        std::memcpy(key_buf, cfg_payload->key, shared::commands::config::MAX_KEY_LEN);
        key_buf[shared::commands::config::MAX_KEY_LEN] = '\0';
        std::string key_str(key_buf);

        if (m_config_callback) {
            ok = m_config_callback(key_str, cfg_payload->value);
        }
    } else if (cmd_hdr.commandID == shared::commands::CommandID::SetActive) {
        constexpr size_t EXPECTED_PAYLOAD = sizeof(shared::commands::CommandsHeader) + sizeof(shared::commands::SetActivePayload);
        if (rx_payload.size() < EXPECTED_PAYLOAD) {
            send_nack(client_fd, rx_hdr.seqNumber, 5);
            return;
        }

        const auto* act_payload = reinterpret_cast<const shared::commands::SetActivePayload*>(rx_payload.data() + sizeof(cmd_hdr));
        if (m_active_callback) {
            ok = m_active_callback(act_payload->active != 0);
        }
    } else if (cmd_hdr.commandID == shared::commands::CommandID::EmergencyStop) {
        constexpr size_t EXPECTED_PAYLOAD = sizeof(shared::commands::CommandsHeader) + sizeof(shared::commands::EmergencyStopPayload);
        if (rx_payload.size() < EXPECTED_PAYLOAD) {
            send_nack(client_fd, rx_hdr.seqNumber, 5);
            return;
        }

        const auto* estop_payload = reinterpret_cast<const shared::commands::EmergencyStopPayload*>(rx_payload.data() + sizeof(cmd_hdr));
        if (m_emergency_callback) {
            ok = m_emergency_callback(estop_payload->reason_code);
        }
    } else {
        send_nack(client_fd, rx_hdr.seqNumber, 4); // Unknown command ID
        return;
    }

    if (ok) {
        send_ack(client_fd, rx_hdr.seqNumber);
    } else {
        send_nack(client_fd, rx_hdr.seqNumber, 6);
    }
}

void ServiceIPCSocketServer::send_ack(int client_fd, uint16_t seqNumber) {
    shared::headers::PacketHeader ack_hdr{};
    ack_hdr.type = shared::headers::HeadersType::Response;
    ack_hdr.headerFlag = 0; // Pas de CRC requis
    ack_hdr.seqNumber = seqNumber;
    ack_hdr.payloadLength = sizeof(uint32_t);

    uint32_t ack_val = static_cast<uint32_t>(shared::responses::ResponseID::Ack);
    auto packet = shared::headers::serializePacket(
        ack_hdr,
        reinterpret_cast<const uint8_t*>(&ack_val),
        sizeof(ack_val)
    );

    send(client_fd, packet.data(), packet.size(), 0);
}

void ServiceIPCSocketServer::send_nack(int client_fd, uint16_t seqNumber, uint16_t nackCode) {
    shared::headers::PacketHeader nack_hdr{};
    nack_hdr.type = shared::headers::HeadersType::Response;
    nack_hdr.headerFlag = 0; // Pas de CRC requis
    nack_hdr.seqNumber = seqNumber;
    nack_hdr.payloadLength = sizeof(uint32_t) + sizeof(shared::responses::nack::NackResponse);

    std::vector<uint8_t> payload(nack_hdr.payloadLength);
    uint32_t nack_id = static_cast<uint32_t>(shared::responses::ResponseID::Nack);
    std::memcpy(payload.data(), &nack_id, sizeof(nack_id));

    shared::responses::nack::NackResponse nack_data{};
    nack_data.explanation_len = 0;
    nack_data.nackCode = nackCode;
    std::memcpy(payload.data() + sizeof(nack_id), &nack_data, sizeof(nack_data));

    auto packet = shared::headers::serializePacket(
        nack_hdr,
        payload.data(),
        payload.size()
    );

    send(client_fd, packet.data(), packet.size(), 0);
}

bool ServiceIPCSocketClient::wait_for_ack(int fd, int timeout_ms) {
    struct pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;

    int p = poll(&pfd, 1, timeout_ms);
    if (p <= 0 || !(pfd.revents & POLLIN)) {
        return false;
    }

    shared::headers::PacketHeader resp_hdr{};
    ssize_t n = recv(fd, &resp_hdr, sizeof(resp_hdr), MSG_PEEK);
    if (n < static_cast<ssize_t>(sizeof(resp_hdr))) {
        return false;
    }

    bool has_crc = (resp_hdr.headerFlag & shared::headers::flags::CRC_ENABLED) != 0;
    size_t expected_resp_len = sizeof(shared::headers::PacketHeader) + resp_hdr.payloadLength + (has_crc ? sizeof(shared::crc::PacketCRC) : 0);

    std::vector<uint8_t> resp_buf(expected_resp_len);
    size_t rx_total = 0;
    while (rx_total < expected_resp_len) {
        int poll_res = poll(&pfd, 1, timeout_ms);
        if (poll_res <= 0) {
            return false;
        }
        ssize_t r = recv(fd, resp_buf.data() + rx_total, expected_resp_len - rx_total, 0);
        if (r <= 0) {
            return false;
        }
        rx_total += r;
    }

    shared::headers::PacketHeader dec_hdr{};
    std::vector<uint8_t> dec_payload;
    uint32_t dec_crc = 0;
    if (!shared::headers::deserializePacket(resp_buf.data(), resp_buf.size(), dec_hdr, dec_payload, dec_crc)) {
        return false;
    }

    if (dec_hdr.type != shared::headers::HeadersType::Response || dec_payload.size() < sizeof(uint32_t)) {
        return false;
    }

    uint32_t resp_id = 0;
    std::memcpy(&resp_id, dec_payload.data(), sizeof(resp_id));
    return (resp_id == static_cast<uint32_t>(shared::responses::ResponseID::Ack));
}

bool ServiceIPCSocketClient::send_set_config(
    const std::string& socket_path,
    shared::commands::ServiceID service_id,
    const std::string& key,
    float value,
    uint16_t seqNumber,
    int timeout_ms
) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(addr.sun_path)) {
        close(fd);
        return false;
    }
    std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return false;
    }

    // Construction du payload : CommandsHeader (4 o) + SetConfigPayload (68 o) = 72 o
    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = service_id;
    cmd_hdr.commandID = shared::commands::CommandID::SetConfigKey;
    cmd_hdr.payloadLen = sizeof(shared::commands::config::SetConfigPayload);

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(shared::commands::config::SetConfigPayload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));
    std::strncpy(reinterpret_cast<char*>(payload_buf.data() + sizeof(cmd_hdr)), key.c_str(), shared::commands::config::MAX_KEY_LEN - 1);
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr) + shared::commands::config::MAX_KEY_LEN, &value, sizeof(float));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seqNumber;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(
        pkt_hdr,
        payload_buf.data(),
        payload_buf.size()
    );

    ssize_t sent = send(fd, packet.data(), packet.size(), 0);
    if (sent != static_cast<ssize_t>(packet.size())) {
        close(fd);
        return false;
    }

    bool res = wait_for_ack(fd, timeout_ms);
    close(fd);
    return res;
}

bool ServiceIPCSocketClient::send_set_active(
    const std::string& socket_path,
    shared::commands::ServiceID service_id,
    bool active,
    uint16_t seqNumber,
    int timeout_ms
) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(addr.sun_path)) {
        close(fd);
        return false;
    }
    std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return false;
    }

    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = service_id;
    cmd_hdr.commandID = shared::commands::CommandID::SetActive;
    cmd_hdr.payloadLen = sizeof(shared::commands::SetActivePayload);

    shared::commands::SetActivePayload act_payload{};
    act_payload.active = active ? 1 : 0;
    act_payload.reserved[0] = act_payload.reserved[1] = act_payload.reserved[2] = 0;

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(act_payload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr), &act_payload, sizeof(act_payload));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seqNumber;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(
        pkt_hdr,
        payload_buf.data(),
        payload_buf.size()
    );

    ssize_t sent = send(fd, packet.data(), packet.size(), 0);
    if (sent != static_cast<ssize_t>(packet.size())) {
        close(fd);
        return false;
    }

    bool res = wait_for_ack(fd, timeout_ms);
    close(fd);
    return res;
}

bool ServiceIPCSocketClient::send_emergency_stop(
    const std::string& socket_path,
    shared::commands::ServiceID service_id,
    uint32_t reason_code,
    uint16_t seqNumber,
    int timeout_ms
) {
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path.size() >= sizeof(addr.sun_path)) {
        close(fd);
        return false;
    }
    std::strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return false;
    }

    shared::commands::CommandsHeader cmd_hdr{};
    cmd_hdr.serviceID = service_id;
    cmd_hdr.commandID = shared::commands::CommandID::EmergencyStop;
    cmd_hdr.payloadLen = sizeof(shared::commands::EmergencyStopPayload);

    shared::commands::EmergencyStopPayload estop_payload{};
    estop_payload.reason_code = reason_code;

    constexpr size_t TOTAL_PAYLOAD_LEN = sizeof(cmd_hdr) + sizeof(estop_payload);
    std::vector<uint8_t> payload_buf(TOTAL_PAYLOAD_LEN, 0);
    std::memcpy(payload_buf.data(), &cmd_hdr, sizeof(cmd_hdr));
    std::memcpy(payload_buf.data() + sizeof(cmd_hdr), &estop_payload, sizeof(estop_payload));

    shared::headers::PacketHeader pkt_hdr{};
    pkt_hdr.type = shared::headers::HeadersType::Command;
    pkt_hdr.headerFlag = shared::headers::flags::REPLY_REQUESTED;
    pkt_hdr.seqNumber = seqNumber;
    pkt_hdr.payloadLength = static_cast<uint32_t>(TOTAL_PAYLOAD_LEN);

    auto packet = shared::headers::serializePacket(
        pkt_hdr,
        payload_buf.data(),
        payload_buf.size()
    );

    ssize_t sent = send(fd, packet.data(), packet.size(), 0);
    if (sent != static_cast<ssize_t>(packet.size())) {
        close(fd);
        return false;
    }

    bool res = wait_for_ack(fd, timeout_ms);
    close(fd);
    return res;
}

} // namespace shared::services::ipc
