/*
------------------------------------------------------------
Compile:
    g++ -O2 -std=c++17 udp_roundtrip_fixed.cpp -o udp_roundtrip_fixed -I /usr/include/eigen3

Run:
    ./udp_roundtrip_fixed

Notes:
    - Micro listens on UDP port 9000 for state x (6 space-separated doubles)
    - Computes u = -K*x
    - Sends u back to PC at 169.254.137.28:9001
------------------------------------------------------------
*/

#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <Eigen/Dense>

int main() {
    const int listen_port = 9000;  // Micro listens here
    const int buffer_size = 1024;

    // ---- Define K (3x6) ----
    Eigen::MatrixXd K(3,6);
    K <<
    4.66705488313515e-06,	-1.36363629843838e-06,	3.96098209388736e-22,	0.00216637769262191,	0.000840447323685129,	7.35693727993496e-20,
2.77991721268819e-06,	7.33444155253651e-07,	6.05699839509399e-22,	0.000150571571893266,	0.00170682586161904,	1.18873969348727e-18,
-3.71173845538396e-22,	-1.12623951370299e-21,	6.24116221637017e-07,	1.62911699574378e-18,	9.12299177441715e-19,	0.00135188823063242;


    std::cout << "Defined control gain matrix K:\n" << K << "\n\n";

    // ---- RX socket (micro listens on 9000) ----
    int rx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock < 0) { perror("socket rx"); return 1; }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(listen_port);
    addr.sin_addr.s_addr = INADDR_ANY;  // any local IP in 169.254.137.0/16

    if (bind(rx_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind");
        close(rx_sock);
        return 1;
    }
    std::cout << "Listening for UDP packets on port " << listen_port << "...\n";

    // ---- TX socket (micro sends back to PC) ----
    int tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock < 0) { perror("socket tx"); return 1; }

    sockaddr_in pc_addr{};
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_port   = htons(9001);  // PC will listen here
    inet_aton("169.254.137.28", &pc_addr.sin_addr);

    // ---- Main loop ----
    char buffer[buffer_size];
    while (true) {
        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t bytes_received = recvfrom(rx_sock, buffer, buffer_size - 1, 0,
                                          reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
        if (bytes_received < 0) {
            perror("recvfrom");
            continue;
        }

        buffer[bytes_received] = '\0';
        std::string message(buffer);

        std::cout << "Received from "
                  << inet_ntoa(sender_addr.sin_addr) << ": "
                  << message << std::endl;

        // ---- Parse message into x ----
        Eigen::VectorXd x(6);
        std::istringstream iss(message);
        for (int i = 0; i < 6; ++i) {
            if (!(iss >> x(i))) {
                std::cerr << "Warning: could not parse 6 doubles\n";
                break;
            }
        }

        // ---- Compute u = -K*x ----
        Eigen::VectorXd u = -K * x;

        // ---- Print u ----
        std::cout << "Computed control u = ["
                  << u(0) << ", " << u(1) << ", " << u(2) << "]\n";

        // ---- Send u back to PC ----
        std::ostringstream oss;
        oss << u(0) << " " << u(1) << " " << u(2);
        std::string u_msg = oss.str();

        ssize_t sent = sendto(tx_sock, u_msg.c_str(), u_msg.size(), 0,
                              reinterpret_cast<sockaddr*>(&pc_addr), sizeof(pc_addr));
        if (sent < 0) {
            perror("sendto");
        } else {
            std::cout << "Sent u back to PC at 169.254.137.28:9001\n";
        }
    }

    close(rx_sock);
    close(tx_sock);
    return 0;
}
