#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
          0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
          -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<Matrix3d> svd(E, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    Vector3d A = svd.singularValues();

    Vector3d A_((A[0] + A[1]) / 2, (A[0] + A[1]) / 2, 0);   //奇异值
    Matrix3d A_m;
    A_m << A_[0], 0, 0,
           0, A_[1], 0,
           0, 0, 0;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2
    // START YOUR CODE HERE
    Matrix3d R_z1(AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix());
    Matrix3d R_z2(AngleAxisd(-M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix());

    Matrix3d t_wedge1 = U * R_z1 * A_m * U.transpose();
    Matrix3d t_wedge2 = U * R_z2 * A_m * U.transpose();

    Matrix3d R1 = U * R_z1 * V.transpose();
    Matrix3d R2 = U * R_z2 * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}