#include <unistd.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace std;

using Trajectory_Type = vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

string groundtruth_path = "/home/fengyixiao/Dark_Blue/SLAM/course/Complete/PA3/groundtruth.txt";
string estimated_path = "/home/fengyixiao/Dark_Blue/SLAM/course/Complete/PA3/estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(Trajectory_Type &g, Trajectory_Type &e);

Trajectory_Type Read_Trajectory(const string &filepath);

int main(int argc, char **argv)
{
    Trajectory_Type groundtruth = Read_Trajectory(groundtruth_path);
    Trajectory_Type estimated = Read_Trajectory(estimated_path);
    assert(!groundtruth.empty() && !estimated.empty());
    assert(groundtruth.size() == estimated.size());

    double rmse = 0;
    for (size_t i = 0; i < estimated.size(); i++){
        Sophus::SE3d SE_rt1 = estimated[i], SE_rt2 = groundtruth[i];
        double error = (SE_rt2.inverse() * SE_rt1).log().norm();
        rmse += error * error;
    }
    rmse = rmse / double(estimated.size());
    rmse = sqrt(rmse);
    cout << "RMSE = " << rmse << endl;

    DrawTrajectory(groundtruth, estimated);

    return 0;
}

Trajectory_Type Read_Trajectory(const string &filepath)
{
    ifstream fin(filepath);
    Trajectory_Type trajectory;
    double time, tx, ty, tz, qx, qy, qz, qw;
    while(!fin.eof())
    {
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3d SE_Rt(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        trajectory.push_back(SE_Rt);
    }
    return trajectory;
}

/*******************************************************************************************/
void DrawTrajectory(Trajectory_Type &g, Trajectory_Type &e) {
    if (g.empty()||e.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < g.size() - 1; i++) {
            glColor3f(1 - (float) i / g.size(), 0.0f, (float) i / g.size());
            glBegin(GL_LINES);
            auto p1 = g[i], p2 = g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < e.size() - 1; i++) {
            glColor3f(1 - (float) i / e.size(), 0.0f, (float) i / e.size());
            glBegin(GL_LINES);
            auto p1 = e[i], p2 = e[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
