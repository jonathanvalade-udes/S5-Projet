#include "FsmEventPipe.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace services {

FsmEventPipe::FsmEventPipe(const std::string& path) : m_path(path) {}

FsmEventPipe::~FsmEventPipe() {
    close_pipe();
}

void FsmEventPipe::close_pipe() {
    if (m_fd >= 0) {
        close(m_fd);
        m_fd = -1;
    }
}

bool FsmEventPipe::check_start_trigger() {
    if (m_fd < 0) {
        struct stat st;
        if (stat(m_path.c_str(), &st) == 0 && S_ISSOCK(st.st_mode)) {
            int sfd = ::socket(AF_UNIX, SOCK_STREAM, 0);
            if (sfd >= 0) {
                int flags = fcntl(sfd, F_GETFL, 0);
                if (flags >= 0) fcntl(sfd, F_SETFL, flags | O_NONBLOCK);
                sockaddr_un addr{};
                addr.sun_family = AF_UNIX;
                strncpy(addr.sun_path, m_path.c_str(), sizeof(addr.sun_path) - 1);
                if (::connect(sfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == 0 || errno == EINPROGRESS) {
                    m_fd = sfd;
                    std::cout << "[FsmEventPipe] Connected to " << m_path << " (waiting for start trigger)" << std::endl;
                } else {
                    close(sfd);
                }
            }
        } else if (stat(m_path.c_str(), &st) == 0 && S_ISFIFO(st.st_mode)) {
            m_fd = open(m_path.c_str(), O_RDWR | O_NONBLOCK);
        }
        if (m_fd < 0) return false;
    }

    char buf[128];
    ssize_t n = read(m_fd, buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        std::string msg(buf);
        std::string lower_msg = msg;
        std::transform(lower_msg.begin(), lower_msg.end(), lower_msg.begin(), ::tolower);
        if (lower_msg.find("trigger_start") != std::string::npos ||
            lower_msg.find("start") != std::string::npos ||
            lower_msg.find("btn1") != std::string::npos ||
            lower_msg.find("button") != std::string::npos) {
            std::cout << "[FsmEventPipe] Event received: " << msg << " -> START!" << std::endl;
            return true;
        }
    } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
        close_pipe();
    }
    return false;
}

} // namespace services
