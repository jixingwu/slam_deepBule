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

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "/home/jixingwu/slam_deepBule/PA5/2/p3d.txt";
string p2d_file = "/home/jixingwu/slam_deepBule/PA5/2/p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    ///TODO load points in to p3d and p2d
    // START YOUR CODE HERE
    std::ifstream in_p3d_file(p3d_file);
    std::ifstream in_p2d_file(p2d_file);
    assert(in_p2d_file.is_open() & in_p3d_file.is_open());

    std::string line;
    double x, y, z;
    while (std::getline(in_p3d_file, line))
    {
        istringstream record(line); record>>x>>y>>z;
        Vector3d point(x, y, z);
        p3d.push_back(point);
    }

    while(std::getline(in_p2d_file, line))
    {
        istringstream record(line); record>>x>>y;
        Vector2d point(x, y);
        p2d.push_back(point);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        //TODO compute cost
        for (int i = 0; i < nPoints; i++) {
            //TODO compute cost for p3d[i] and p2d[i]
            /// START YOUR CODE HERE

            //见书p87
            Vector4d P_ = T_esti.matrix()*Vector4d(p3d[i](0,0),p3d[i](1,0),p3d[i](2,0),1); //T*p3d
            Vector3d P_c  = K * Vector3d(P_(0,0), P_(1,0), P_(2,0));//u=K*P
            Vector2d e = p2d[i] - Vector2d(P_c(0,0)/P_c(2,0), P_c(1,0)/P_c(2,0));//e=p2d-Pc

            cost +=e.squaredNorm()/2;//cost=1/2 * e^2, 使用e的二范数

            cout<<"p: "<<P_(1,0)<<endl<<P_(1);


	    // END YOUR CODE HERE

	    //TODO compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double X=P_(0,0), Y=P_(1,0), Z=P_(2,0);
            J(0,0)=fx/Z;
            J(0,1)=0;
            J(0,2)=-fx*x/Z/Z;
            J(0,3)=-fx*X*Y/Z/Z;
            J(0,4)=fx+fx*X*X/Z/Z;
            J(0,5)=-fx*Y/Z;

            J(1,0)=0;
            J(1,1)=fy/Z;
            J(1,2)=-fy*Y/Z/Z;
            J(1,3)=-fy-fy*Y*Y/Z/Z;
            J(0,4)=fy*Y*X/Z/Z;
            J(0,5)=fy*X/Z;

            J=-J;


	    // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    //TODO solve dx
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

        // TODO update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx) * T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
