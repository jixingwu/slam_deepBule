#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;
using namespace cv;

// path to trajectory file


// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void AlignTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    string trajectory_file = "trajectory.txt";
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
    vector<Point3f> t_e;
    vector<Point3f> t_g;
    Matrix3d R;
    Vector3d t;


    // implement pose reading code
    std::ifstream infile("/home/jixingwu/slam_deepBule/PA5/4/compare.txt");
    assert(infile.is_open());

    //read data
    double te,txe,tye,tze,qxe,qye,qze,qwe,tg,txg,tyg,tzg,qxg,qyg,qzg,qwg;

    std::string line;
    while(std::getline(infile,line))//getline(fin,line)
    {
        istringstream record(line);    //从string读取数据
        record>>te>>txe>>tye>>tze>>qxe>>qye>>qze>>qwe
              >>tg>>txg>>tyg>>tzg>>qxg>>qyg>>qzg>>qwg;

        Eigen::Vector3d te(txe,tye,tze);
        t_e.push_back(Point3d(txe,tye,tze));
        Eigen::Quaterniond qe = Eigen::Quaterniond(qwe,qxe,qye,qze).normalized();  //四元数的顺序要注意
        Sophus::SE3 SE3_qt_e(qe,te);
        poses_e.push_back(SE3_qt_e);

        Eigen::Vector3d tg(txg,tyg,tzg);
        t_g.push_back(Point3d(txg,tyg,tzg));
        Eigen::Quaterniond qg = Eigen::Quaterniond(qwg,qxg,qyg,qzg).normalized();
        Sophus::SE3 SE3_qt_g(qg,tg);
        poses_g.push_back(SE3_qt_g);
    }

    //icp_svd
    int N = t_e.size();
    Point3f p1, p2;
    for (int i = 0; i < N; ++i) {
        p1+=t_e[i];
        p2+=t_g[i];
    }
    p1=Point3f(Vec3f(p1)/N); p2=Point3f(Vec3f(p2)/N);
    vector<Point3f> q1(N), q2(N);
    for (int i = 0; i < N; ++i) {
        q1[i]=t_e[i] - p1;
        q2[i]=t_g[i] - p2;
    }

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();

    }
    cout<<"W= "<<W<<endl;

    //SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U= "<<U<<endl;
    cout<<"V= "<<V<<endl;

    R = U*(V.transpose());//p1=R_12*p2;,注意R的意义，p2到p1的旋转关系
    t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R*Eigen::Vector3d(p2.x, p2.y, p2.z);
    

    // draw trajectory in pangolin
    AlignTrajectory(poses_e, poses_g);
    return 0;
}

/*******************************************************************************************/
void AlignTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g)
{
    if (poses_e.empty() || poses_g.empty()) {
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


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses_e.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_e.size(), 0.0f, (float) i / poses_e.size());
            glBegin(GL_LINES);
            auto p1 = poses_e[i], p2 = poses_e[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < poses_g.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_g.size(), 0.0f, (float) i / poses_g.size());
            glBegin(GL_LINES);
            auto p1 = poses_g[i], p2 = poses_g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}
