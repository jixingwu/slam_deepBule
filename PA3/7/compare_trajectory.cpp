#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file


// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
double CompareTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

//    string estimated_file = "/home/jixingwu/Thiredpart/slam_deepBule/PA3/7/estimated.txt";
//    string groundtruth_file = "/home/jixingwu/Thiredpart/slam_deepBule/PA3/7/groundtruth.txt";
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth_poses;

    // implement pose reading code
    std::ifstream estimated_infile("/home/jixingwu/Thiredpart/slam_deepBule/PA3/7/estimated.txt");
    std::ifstream groundtruth_infile("/home/jixingwu/Thiredpart/slam_deepBule/PA3/7/groundtruth.txt");
    assert(estimated_infile.is_open());

    double t,tx,ty,tz,qx,qy,qz,qw;

    std::string line;
    while(std::getline(estimated_infile, line))//getline(fin,line)
    {
        istringstream record(line);    //从string读取数据
        record>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Eigen::Vector3d ts(tx,ty,tz);
        Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();  //四元数的顺序要注意
        Sophus::SE3 SE3_qt(q,ts);
        estimated_poses.push_back(SE3_qt);
    }


    while(std::getline(groundtruth_infile, line))
    {
        istringstream record(line);    //从string读取数据
        record>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        Eigen::Vector3d ts(tx,ty,tz);
        Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();  //四元数的顺序要注意
        Sophus::SE3 SE3_qt(q,ts);
        groundtruth_poses.push_back(SE3_qt);
    }


    double RMSE = CompareTrajectory(estimated_poses, groundtruth_poses);
    cout<<"RMSE: "<<RMSE<<endl;
    return 0;

}

/*******************************************************************************************/
double CompareTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth_poses)
{
    double RMSE = 0;
    vector<double> error;
    Eigen::Matrix<double, 6, 1> se3;
    for (size_t i = 0; i < estimated_poses.size(); ++i) {
        se3 = (groundtruth_poses[i].inverse()*estimated_poses[i]).log();
        error.push_back(se3.squaredNorm());//二范数
    }

    for (int j = 0; j < estimated_poses.size(); ++j) {
        RMSE += error[j];
    }
    RMSE /= double(error.size());
    RMSE = sqrt(RMSE);
    return RMSE;
}



