/*
Build (MSVC, Dev Cmd Prompt):
    cl /O2 /EHsc udp_hcw_discrete_txrx.cpp ws2_32.lib
 
Run:
    udp_hcw_discrete_txrx.exe
 
Behavior:
  - Sends x_k over UDP to Jetson (IP/port below).
  - Listens on UDP port 9001 for ASCII control "u0 u1 u2".
  - Uses the most recent u in x_{k+1} = Ad x_k + Bd u_k.
  - If no control arrives before timeout, keeps previous u.
*/
 
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h> // for Sleep
#include <Eigen/Dense>
 
#pragma comment(lib, "ws2_32.lib")
 
int main() {
    // ---- Winsock init (minimal checks) ----
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) return 1;
 
    // ---- TX socket to Jetson ----
    SOCKET tx_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock == INVALID_SOCKET) return 1;
 
    sockaddr_in jetsonAddr{};
    jetsonAddr.sin_family = AF_INET;
    jetsonAddr.sin_port = htons(9000);                         // Jetson RX port
    inet_pton(AF_INET, "169.254.137.50", &jetsonAddr.sin_addr); // Jetson IP
 
    // ---- RX socket for controls on port 9001 ----
    SOCKET rx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (rx_sock == INVALID_SOCKET) return 1;
 
    sockaddr_in rx_addr{};
    std::memset(&rx_addr, 0, sizeof(rx_addr));
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_port = htons(9001);
    rx_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(rx_sock, reinterpret_cast<sockaddr*>(&rx_addr), sizeof(rx_addr)) == SOCKET_ERROR) return 1;
 
    // Receive timeout so the loop doesn't block forever (ms)
    DWORD rx_timeout_ms = 5000; // 5 s
    setsockopt(rx_sock, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&rx_timeout_ms),
               sizeof(rx_timeout_ms));
 
    // ---- Initial state and matrices ----
    Eigen::Matrix<double,6,1> x_now;
    x_now << 20, 20, 20, 0.00930458, -0.0467472, 0.00798343;
 
    Eigen::Vector3d u_now = Eigen::Vector3d::Zero(); // until first packet arrives
 
    Eigen::Matrix<double,6,6> Ad;
      Ad <<
        1.00155466587491, 0.0, 0.0, 29.9948176013589, 0.965773759516463, 0.0,
        -3.33697130953994e-05, 1.0, 0.0, -0.965773759516463, 29.9792704054355, 0.0,
        0.0, 0.0, 0.999481778041697, 0.0, 0.0, 29.9948176013589,
        0.000103635438932749, 0.0, 0.0, 0.999481778041697, 0.0643793557783298, 0.0,
        -3.33685601317627e-06, 0.0, 0.0, -0.0643793557783298, 0.997927112166788, 0.0,
        0.0, 0.0, -3.45451463109164e-05, 0.0, 0.0, 0.999481778041697;

    Eigen::MatrixXd Bd(6, 3);
    Bd << 29.9948176013589, 0.965773759516463, 0,
        -0.965773759516463, 29.9792704054355, 0,
        0, 0, 29.9948176013589,
        0.999481778041697, 0.0643793557783298, 0,
        -0.0643793557783298, 0.997927112166788, 0,
        0, 0, 0.999481778041697;
 
    std::cout << "Listening for control on UDP port 9001 (format: \"u0 u1 u2\").\n";

    //.txt File Formatting
    std::ofstream csvFile("Couting.csv"); //making data file named "Couting.csv"
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "");//declaring csv format
    //1 - make a data file (overright existing file blank mod for leaving open to write into it in delcaration-ascii.csv data file) 
 
    // ---- Main loop ----
    for (int i = 0; i < 91; ++i) {
        //Time
        double t = i * 1.0;
        //2 - write (append) 'xt' to data file (write statements for append to)
        csvFile << "t:" << ","; //label
        csvFile << t << ",";
        csvFile << "xt:" << ","; // label
        csvFile << x_now.format(CSVFormat) << ",";
        csvFile << "u:" << ","; // label
        csvFile << u_now.format(CSVFormat) << "\n";
        //3 - forcing write without closing
        csvFile.flush();

        // Propagate with current u
        Eigen::Matrix<double,6,1> x_next = Ad * x_now + Bd * u_now;
        x_now = x_next;
 
        // Send state to Jetson
        std::ostringstream oss;
        oss << x_now(0) << ' ' << x_now(1) << ' ' << x_now(2) << ' '
        << x_now(3) << ' ' << x_now(4) << ' ' << x_now(5);

        std::string msg = oss.str();
        sendto(tx_sock, msg.c_str(), static_cast<int>(msg.size()), 0, reinterpret_cast<sockaddr*>(&jetsonAddr), sizeof(jetsonAddr));
        std::cout << i << ": x = " << msg << "\n";

        // Minimal control receive (timeout → keep old u)
        char buffer[1024];
        sockaddr_in sender_addr{};
        int sender_len = sizeof(sender_addr);
        int n = recvfrom(rx_sock, buffer, sizeof(buffer) - 1, 0,
                         reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
 
        if (n > 0) {
            buffer[n] = '\0';
            std::istringstream iss(buffer);
            double u0, u1, u2;
            if (iss >> u0 >> u1 >> u2) {
                u_now << u0, u1, u2; // use for NEXT step
                std::cout << "u = [" << u_now(0) << ", " << u_now(1) << ", " << u_now(2) << "]\n";
            } // else: ignore malformed packet
        } else {
            std::cout << "no control received (timeout) keeping previous u \n";
        }
       

        // Slow loop so a control is likely to arrive
        Sleep(100); // ms
    }
 
    // ---- Cleanup ----
    closesocket(rx_sock);
    closesocket(tx_sock);
    WSACleanup();
    return 0;
}
