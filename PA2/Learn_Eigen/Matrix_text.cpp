#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

#define MATRIX_SIZE 100

int main(int argc, char** argv)
{
    Eigen::MatrixXd G = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::MatrixXd A = G.transpose() * G;

    //Judgement for Positive Definite Matrix
    while(A.determinant()==0) {
        G = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
        A = G.transpose() * G;
    }

    Eigen::MatrixXd b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    //Normal Inverse
    clock_t start_c = clock();
    Eigen::MatrixXd x;
    x = A.inverse() * b;
    cout << "time use in Normal Inverse is " << 1000*(clock() - start_c)/(double)CLOCKS_PER_SEC << "ms" << endl << x.transpose() << endl;

    //QR Decomposition
    start_c = clock();
    x = A.colPivHouseholderQr().solve(b);
    cout << "time use in QR Decomposition is " << 1000*(clock() - start_c)/(double)CLOCKS_PER_SEC << "ms" << endl << x.transpose() << endl;

    //Cholesky Decomposition
    start_c = clock();
    x = A.llt().solve(b);
    //x = A.ldlt().solve(b);
    cout << "time use in Cholesky Decomposition is " << 1000*(clock() - start_c)/(double)CLOCKS_PER_SEC << "ms" << endl << x.transpose() << endl;

    return 0;
}
