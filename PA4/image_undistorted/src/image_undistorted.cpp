#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "/home/fengyixiao/Dark_Blue/SLAM/course/Complete/PA4/image_undistorted/test.png";

int main(int argc, char **argv) {

    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file,CV_8UC1);
    cv::imshow("original", image);
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图

    // 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)  //行
        for (int u = 0; u < cols; u++)  //列
        {
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * r * r + k2 * pow(r, 4)) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * (1 + k1 * r * r + k2 * pow(r, 4)) + p1 * (r * r + 2 * y * y) + 2 * p2 * x *y;
            double u_distorted = x_distorted * fx + cx, v_distorted = y_distorted * fy + cy;

            // 赋值 (最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistorted.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistorted.at<uchar>(v, u) = 0;
            }
        }

    // 画图去畸变后图像
    cv::imshow("image undistorted", image_undistorted);
    cv::waitKey();

    return 0;
}