#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>


//Function to Save Matrices as Header Files
void MatExport_H(const Eigen::MatrixXd& m,const std::string& filename){
    std::ofstream file(filename, std::ios::binary | std::ios::trunc);
    int rows = m.rows();
    int cols = m.cols();
    file.write(reinterpret_cast<char*>(&rows), sizeof(int));
    file.write(reinterpret_cast<char*>(&cols), sizeof(int));
    file.write(reinterpret_cast<const char*>(m.data()), rows * cols * sizeof(double));
}

//Function for Block Diagonal Matrices
Eigen::MatrixXd blkdia(const std::vector<Eigen::MatrixXd>& mats) {
    int row = 0, col = 0; //row and col vars
    for (const auto& m : mats){ 
        row += m.rows(); //defining dims
        col += m.cols(); 
        }
        
        Eigen::MatrixXd bd = Eigen::MatrixXd::Zero(row,col);
        int r = 0; //index vars
        int c = 0;

        //populating block diagonals
        for (const auto& m : mats) {
            bd.block(r,c, m.rows(), m.cols()) = m;
            r += m.rows();
            c += m.cols();
        }
        
    return bd;
} 

//Function for Kronecker Product:
Eigen::MatrixXd Kron(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B){
    int rA = A.rows(), cA = A.cols(); //defining dims
    int rB = B.rows(), cB = B.cols();

    Eigen::MatrixXd prod(rA * rB, cA * cB); //assigning dims

    for (int i = 0; i < rA; ++i){
        for (int j = 0; j < cA; ++j){
            prod.block(i*rB, j*cB, rB, cB) = A(i,j) * B; //Making and assigning kron blocks
        }
    }
    return prod; //prod is kron prod
}

//Matrix Power Function: 
Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int power)
{
    if (power == 0)
        return Eigen::MatrixXd::Identity(A.rows(), A.cols());

    Eigen::MatrixXd pow = A;
    for(int k = 1; k < power; ++k)
        pow *= A;

    return pow;
}

int main() {
    using namespace std;
    using namespace Eigen;

    //Asking for horizon:
    int N; //prediction horizon
    std::cout << "Enter prediciton horizon: ";
    std::cin >> N;

    //Hard coding discretized matrices
    Eigen::MatrixXd Ad(6,6);
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

    //State weights:
    Eigen::MatrixXd Q(6,6);
    Q <<  10, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 
            0, 0, 0, 0, 1, 0, 
            0, 0, 0, 0, 0, 1;
    
    Q = 1e-1*Q;

    Eigen::MatrixXd R(3,3);
    R <<  1, 0, 0,
          0, 1, 0,
          0, 0, 1;
    
    R = 2e6 * R;

    //Dimensions:
    int n = Ad.rows();
    int m = Bd.cols();

    //Converging on P
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n,n);
    double eps = std::numeric_limits<double>::epsilon();
    for (int k = 0; k < 1000; ++k) {
        Eigen::MatrixXd Chunk = (R+Bd.transpose()*P*Bd);
        //why did they define inverse as lu_inv in quadprog++...
        #undef inverse
        #undef solve //omg 
        Eigen::MatrixXd Chunk_inv = Chunk.inverse();
        Eigen::MatrixXd P_new = Ad.transpose()*P*Ad - (Ad.transpose()*P*Bd)*Chunk_inv*(Bd.transpose()*P*Ad)+Q;
        if ((P_new-P).norm() <=  eps){
            P = P_new;
            break;
        }
        P = P_new;
    }
    
    //Matrices to simplify generation
    Eigen::MatrixXd PSI = Eigen::MatrixXd::Zero(N*n, n);
    Eigen::MatrixXd OMEGA = Eigen::MatrixXd::Zero(n*N, m*N);

    //U TOLERANCE:
    //Asking for horizon:
    double umax; //prediction horizon
    std::cout << "Enter maximum u: ";
    std::cin >> umax;
       

    for (int i = 1; i<=N ; ++i){
        //A^(i) calc:
        //PSI block for indexing
        PSI.block(i*n-n, 0, n, PSI.cols()) = matrixPower(Ad,i);

        for(int j = 1; j<= i; ++j){
            //A^(i-j) calc:
            //OMEGA BLOCK
            OMEGA.block(i*n-n,j*m -m, n, m) = matrixPower(Ad, i-j)*Bd;
        }   
    }
    
    Eigen::MatrixXd IL1 = Eigen::MatrixXd::Identity(N,N); 
    Eigen::MatrixXd IL2 = Eigen::MatrixXd::Identity(N-1,N-1); 
    Eigen::MatrixXd L1 = Kron(IL1,R); 
    Eigen::MatrixXd L2 = blkdia({Kron(IL2,Q),P}); 

    //Generating matrices 
    Eigen::MatrixXd QQ = 2*L1.eval() + 2*OMEGA.transpose()*L2*OMEGA.eval(); 
    Eigen::MatrixXd H1 = 2*PSI.transpose()*L2*OMEGA.eval();

    //Aineq
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(2*N*m, m*N);
    Aineq.block(0,0,N*m, N*m) = -1*Eigen::MatrixXd::Identity(N*m,N*m);
    Aineq.block(N*m,0, N*m, N*m) = Eigen::MatrixXd::Identity(N*m,N*m);
    Eigen::MatrixXd CI_e = -1*Aineq.transpose();

    //Bineq
    Eigen::MatrixXd Bineq = Eigen::MatrixXd::Zero(2*N*m, 1);
    Bineq.block(0,0,N*m, 1) = umax*Eigen::VectorXd::Ones(N*m);
    Bineq.block(N*m,0,N*m, 1) = umax*Eigen::VectorXd::Ones(N*m);

    //Exporting to .hpp files function calls
    MatExport_H(Aineq,"../Aineq.bin");
    MatExport_H(Bineq,"../Bineq.bin");
    MatExport_H(QQ,"../QQ.bin");
    MatExport_H(H1,"../H1.bin");
}
