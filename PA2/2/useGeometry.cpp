//
// Created by jixingwu on 2019/11/1.
//
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Quaterniond q1 = Quaterniond(0.55,0.3,0.2,0.2).normalized();   //定义并归一化四元数
    Quaterniond q2 = Quaterniond(-0.1,0.3,-0.7,0.2).normalized();
    Vector3d t1 ,t2,p1,p,p2;
    t1 << 0.7,1.1,0.2;
    t2 << -0.1,0.4,0.8;
    p1 << 0.5,-0.1,0.2;

    Isometry3d T_cw1 = Isometry3d::Identity();
    T_cw1.rotate ( q1 );
    T_cw1.pretranslate ( t1 );

    Isometry3d T_cw2 = Isometry3d::Identity();
    T_cw2.rotate ( q2 );
    T_cw2.pretranslate ( t2 );



    p = T_cw1.inverse() *p1;   //不能用转置，因为这里不是纯旋转，见书42页
    p2 = T_cw2 *p ;

    cout<<"p2 = "<<p2.transpose()<<endl;

    return 0;
}
