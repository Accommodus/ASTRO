#include <iostream>
#include <Eigen/Dense>
#include "QQ.hpp"
#include "H1.hpp"
#include "Aineq.hpp"
#include "Bineq.hpp"
#include "QuadProg++.hh"

//horizon = 15;


//Eigen → QuadProg++
quadprogpp::Matrix<double> eigenToQP(const Eigen::MatrixXd& M)
{
    quadprogpp::Matrix<double> Q(M.rows(), M.cols());
    for (int i = 0; i < M.rows(); ++i)
        for (int j = 0; j < M.cols(); ++j)
            Q[i][j] = M(i,j);
    return Q;
}

quadprogpp::Vector<double> eigenToQP(const Eigen::VectorXd& v)
{
    quadprogpp::Vector<double> q(v.size());
    for (int i = 0; i < v.size(); ++i)
        q[i] = v(i);
    return q;
}

// QuadProg++ → Eigen
Eigen::VectorXd qpToEigen(const quadprogpp::Vector<double>& v)
{
    Eigen::VectorXd e(v.size());
    for (int i = 0; i < v.size(); ++i)
        e(i) = v[i];
    return e;
} 

int main() {
    //matrices r hard coded included header files
    using namespace std;
    using namespace Eigen;

    //givens:
    double Re = 6371.0;
    double mu = 398600.4;
    double Ro = 650.0;
    double n  = std::sqrt(mu / std::pow(Re + Ro, 3));
    double Ts = 30.0;
   

    //matrices:
    Eigen::Matrix<double, 6, 6> Ad;
    Ad <<
        1.00155466587491,        0.0,                     0.0,  29.9948176013589,   0.965773759516463,  0.0,
       -3.33697130953994e-05,    1.0,                     0.0,  -0.965773759516463, 29.9792704054355,   0.0,
        0.0,                     0.0,  0.999481778041697,  0.0,                     0.0, 29.9948176013589,
        0.000103635438932749,    0.0,                     0.0,  0.999481778041697,  0.0643793557783298, 0.0,
       -3.33685601317627e-06,    0.0,                     0.0, -0.0643793557783298, 0.997927112166788,  0.0,
        0.0,                     0.0, -3.45451463109164e-05, 0.0,                   0.0, 0.999481778041697;
    Eigen::MatrixXd Bd(6,3);
    Bd << 29.9948176013589,  0.965773759516463, 0,
        -0.965773759516463, 29.9792704054355,  0,
         0,                  0,                 29.9948176013589,
         0.999481778041697,  0.0643793557783298, 0,
        -0.0643793557783298, 0.997927112166788,  0,
         0,                   0,                 0.999481778041697;
    Eigen::MatrixXd C_pos(3,6), C_vel(3,6);
    C_pos << MatrixXd::Identity(3,3), MatrixXd::Zero(3,3);
    C_vel << MatrixXd::Zero(3,3), MatrixXd::Identity(3,3);

    //simulation:
    VectorXd X(6);
    X << 20, 20, 20, 0, 0, 0;

    double tol_pos = 1e-4;
    double tol_vel = 1e-4;
    double u_max   = 0.01;

    std::vector<VectorXd> x_traj, u_traj;
    

    //prop loop
    for (int i = 0; i < 1000; ++i)
    {

    //Build QP in Eigen
    Eigen::MatrixXd G_e   = QQ;    
    Eigen::VectorXd g0_e  = H1.transpose()*X;
    Eigen::MatrixXd CI_e  = -Aineq.transpose();
    Eigen::VectorXd ci0_e = Bineq;

    
    //Quadprog inputs
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
    Eigen::VectorXd U = Utot.segment(0,3);
     cout << "computed control:" << U.transpose() << endl;
        X = Ad * X + Bd * U;
        
     cout << "X:" << X.transpose() << endl;
        if ( (C_pos * X).norm() < tol_pos &&
             (C_vel * X).norm() < tol_vel )
        {
            std::cout << "Docking achieved at t = "
                      << i * Ts << " s\n";
            break;
        }
    }

    std::cout << "Final state:\n" << X.transpose() << std::endl;
    return 0;

}


