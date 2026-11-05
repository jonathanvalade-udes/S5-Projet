// L'arret d'urgence envoye par le site doit atteindre le FSM ET les moteurs.
// Si seul Motors est prevenu, le FSM continue de commander les moteurs et le robot repart au cycle suivant.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "ProtocolHandler.hpp"
#include "TcpClient.hpp"
#include "commands/emergency_stop.hpp"
#include "commands/global_commands.hpp"
#include "crc/crc_global.hpp"
#include "header/header.hpp"

namespace {

constexpr int TEST_PORT = 9293;

// Serveur factice : accepte une connexion et lit tout ce qui arrive pendant ~600 ms
std::vector<uint8_t> collect_bytes(std::atomic<bool>& listening) {
    std::vector<uint8_t> received;
    int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(TEST_PORT);
    assert(::bind(srv, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    ::listen(srv, 1);
    listening.store(true);

    pollfd pfd{srv, POLLIN, 0};
    if (::poll(&pfd, 1, 3000) > 0) {
        int cli = ::accept(srv, nullptr, nullptr);
        const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
        uint8_t buf[512];
        while (std::chrono::steady_clock::now() < end) {
            pollfd cp{cli, POLLIN, 0};
            if (::poll(&cp, 1, 50) > 0) {
                ssize_t n = ::recv(cli, buf, sizeof(buf), 0);
                if (n <= 0) break;
                received.insert(received.end(), buf, buf + n);
            }
        }
        ::close(cli);
    }
    ::close(srv);
    return received;
}

// Liste les services vises par des commandes EmergencyStop dans le flux recu
std::vector<shared::commands::ServiceID> emergency_targets(const std::vector<uint8_t>& stream) {
    std::vector<shared::commands::ServiceID> targets;
    size_t pos = 0;
    constexpr size_t HDR = sizeof(shared::headers::PacketHeader);
    while (pos + HDR <= stream.size()) {
        shared::headers::PacketHeader hdr{};
        std::memcpy(&hdr, stream.data() + pos, HDR);
        const size_t crc = (hdr.headerFlag & shared::headers::flags::CRC_ENABLED) ? sizeof(shared::crc::PacketCRC) : 0;
        const size_t total = HDR + hdr.payloadLength + crc;
        if (pos + total > stream.size() || hdr.payloadLength < sizeof(shared::commands::CommandsHeader)) break;
        shared::commands::CommandsHeader cmd{};
        std::memcpy(&cmd, stream.data() + pos + HDR, sizeof(cmd));
        if (hdr.type == shared::headers::HeadersType::Command &&
            cmd.commandID == shared::commands::CommandID::EmergencyStop) {
            targets.push_back(cmd.serviceID);
        }
        pos += total;
    }
    return targets;
}

bool contains(const std::vector<shared::commands::ServiceID>& v, shared::commands::ServiceID s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

} // namespace

int main() {
    for (const char* cmd : {"emergency_stop", "stop"}) {
        std::atomic<bool> listening{false};
        std::vector<uint8_t> received;
        std::thread server([&] { received = collect_bytes(listening); });
        while (!listening.load()) std::this_thread::sleep_for(std::chrono::milliseconds(5));

        telemetry::TcpClient client;
        telemetry::ProtocolHandler proto;
        assert(client.connect_to("127.0.0.1", TEST_PORT));
        assert(proto.send_command(client, cmd, 7));
        client.disconnect();
        server.join();

        const auto targets = emergency_targets(received);
        std::cout << "commande \"" << cmd << "\" : " << targets.size() << " arret(s) d'urgence envoye(s)" << std::endl;
        assert(contains(targets, shared::commands::ServiceID::FSM));      // le FSM doit etre prevenu (sinon il redemarre les moteurs)
        assert(contains(targets, shared::commands::ServiceID::MOTORS));   // et les moteurs s'arretent tout de suite
    }
    std::cout << "test_emergency_stop : tous les tests passent" << std::endl;
    return 0;
}
