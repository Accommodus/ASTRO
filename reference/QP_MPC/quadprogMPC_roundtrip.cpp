#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <Eigen/Dense>
#include "QQ.hpp"
#include "H1.hpp"
#include "Aineq.hpp"
#include "Bineq.hpp"
#include "QuadProg++.hh"

//Conversion functions:
//matrix conv to qp
quadprogpp::Matrix<double> eigenToQP(const Eigen::MatrixXd& M)
{
    quadprogpp::Matrix<double> Q(M.rows(), M.cols());
    for (int i = 0; i < M.rows(); ++i)
        for (int j = 0; j < M.cols(); ++j)
            Q[i][j] = M(i,j);
    return Q;
}
//vector conv to qp
quadprogpp::Vector<double> eigenToQP(const Eigen::VectorXd& v)
{
    quadprogpp::Vector<double> q(v.size());
    for (int i = 0; i < v.size(); ++i)
        q[i] = v(i);
    return q;
}
//vector conv to eigen
Eigen::VectorXd qpToEigen(const quadprogpp::Vector<double>& v)
{
    Eigen::VectorXd e(v.size());
    for (int i = 0; i < v.size(); ++i)
        e(i) = v[i];
    return e;
} 

int main() {
    //Note: qp matrices are hard coded included header files
    using namespace std;
    using namespace Eigen;

    //Socket stuff:
    const int listen_port = 9000;  // Micro listens here
    const int buffer_size = 1024;
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
        Eigen::VectorXd X(6);
        std::istringstream iss(message);
        for (int i = 0; i < 6; ++i) {
            if (!(iss >> X(i))) {
                std::cerr << "Warning: could not parse 6 doubles\n";
                break;
            }
        }
    
        //prop loop

        // ----- Build QP in Eigen -----
        Eigen::MatrixXd G_e   = QQ;    
        Eigen::VectorXd g0_e  = H1.transpose()*X;
        Eigen::MatrixXd CI_e  = -Aineq.transpose();
        Eigen::VectorXd ci0_e = Bineq;
        
        //Quadprog inputs (cannot use eigen)
        quadprogpp::Matrix<double> G   = eigenToQP(G_e);
        quadprogpp::Vector<double> g0  = eigenToQP(g0_e);
        quadprogpp::Matrix<double> CE(45,0); //no inequality constraints
        quadprogpp::Vector<double> ce0(0); //no inequality constraints
        quadprogpp::Matrix<double> CI  = eigenToQP(CI_e);
        quadprogpp::Vector<double> ci0 = eigenToQP(ci0_e);
        quadprogpp::Vector<double> xqp(45);
        //making x for qp all 0.0 without eigen
        for(int i=0;i<45;i++) xqp[i] = 0.0;

        // Solve
        double cost = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, xqp);

        // Convert back
        Eigen::VectorXd Utot = qpToEigen(xqp);
        Eigen::VectorXd u = Utot.segment(0,3); //gathering control from augmented vector
        
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


