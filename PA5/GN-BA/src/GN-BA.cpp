//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "/home/fengyixiao/Dark_Blue/SLAM/course/Complete/PA5/GN-BA/p3d.txt";
string p2d_file = "/home/fengyixiao/Dark_Blue/SLAM/course/Complete/PA5/GN-BA/p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream f_p2d(p2d_file), f_p3d(p3d_file);
    while(!f_p2d.eof() || !f_p3d.eof())
    {
        double x_2, y_2;    double x_3, y_3, z_3;
        f_p2d >> x_2 >> y_2;
        f_p3d >> x_3 >> y_3 >> z_3;

        Vector2d point_2d(x_2, y_2);
        Vector3d point_3d(x_3, y_3, z_3);
        p2d.push_back(point_2d); p3d.push_back(point_3d);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3d T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();  //高斯牛顿J(x).transpose()*J(x)  (J(x)是2*6矩阵)
        Vector6d b = Vector6d::Zero();  //-J(x)*e   e是二维的   故b是6*1矩阵

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Vector3d pc = T_esti * p3d[i];      //从3D坐标到相机坐标系[X',Y',Z']
            double z = 1 / pc[2]; double z_ = 1 / (pc[2] * pc[2]);
            Vector2d proj_2d(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);
            Vector2d e = p2d[i] - proj_2d;
            cost += e.squaredNorm();
	        // END YOUR CODE HERE

	        // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J << - fx * z, 0, fx * pc[0] * z_, fx * pc[0] * pc[1] * z_, - fx - fx * pc[0] * pc[0] * z_, fx * pc[1] * z,
                 0, - fy * z, fy * pc[1] * z_, fy + fy * pc[1] * pc[1] * z_, - fy * pc[0] * pc[1] * z_, - fy * pc[0] * z;
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx
        Vector6d dx;
        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3d::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
