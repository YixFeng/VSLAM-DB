#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

int main(int argc, char** argv)
{
    //小萝卜一号
    Eigen::Quaterniond Q1(0.55, 0.3, 0.2, 0.2); //赋值实部在前，存储xyzw
    Q1.normalize();
    Eigen::Isometry3d I1 = Eigen::Isometry3d::Identity();
    Eigen::Vector3d T1(0.7,1.1,0.2);
    I1.rotate(Q1);
    I1.pretranslate(T1);
    //cout << "Carrot 1: \n" << I1.matrix() << endl;

    //小萝卜二号
    Eigen::Quaterniond Q2(-0.1, 0.3, -0.7, 0.2);
    Q2.normalize();
    Eigen::Isometry3d I2 = Eigen::Isometry3d::Identity();
    Eigen::Vector3d T2(-0.1, 0.4, 0.8);
    I2.rotate(Q2);
    I2.pretranslate(T2);
    //cout << "Carrot 2: \n" << I2.matrix() << endl;

    Eigen::Isometry3d I_t = I2 * I1.inverse();
    //cout << "C1 to C2: \n" << I_t.matrix() << endl;

    Eigen::Vector3d p1(0.5 ,-0.1 ,0.2);
    Eigen::Vector3d p2 = I_t * p1;
    cout << "p2 is: " << p2.transpose();

    return  0;
}

