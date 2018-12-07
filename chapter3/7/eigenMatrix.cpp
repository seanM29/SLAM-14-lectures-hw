#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50


int main( int argc, char** argv )
{

  //Eigen::Vector4d q_c_1 ;
  Eigen::Quaterniond q_c_1(0.35, 0.2, 0.3,0.1 );


  Eigen::Matrix<double,3,3> r_c12w =q_c_1.normalized().toRotationMatrix();
  Eigen::Matrix<double,4,4> pose_w2c1 = Eigen::Matrix4d::Zero();
  pose_w2c1.block<3,3>(0,0) = r_c12w.transpose();
  Eigen::Vector3d t_c12w;
  t_c12w <<  0.3,0.1,0.1;

  Eigen::Vector3d t_w2c1 = -r_c12w.transpose()*t_c12w;
  pose_w2c1.block<3,1>(0,3) = t_w2c1;
  pose_w2c1(3,3) =1;
  std::cout<<pose_w2c1<<std::endl;

  
  Eigen::Quaterniond q_c_2(-0.5,0.4,-0.1,0.2 );
  Eigen::Matrix<double,3,3> r_c22w =q_c_2.normalized().toRotationMatrix();
  
  Eigen::Matrix<double,4,4> pose_c22w = Eigen::Matrix4d::Zero();
  pose_c22w.block<3,3>(0,0) = r_c22w;
  Eigen::Vector3d t_c22w;
  t_c22w << -0.1,0.5,0.3;
  pose_c22w.block<3,1>(0,3) = t_c22w;
  pose_c22w(3,3) =1;

  std::cout<<pose_c22w<<std::endl;

  Eigen::Matrix4d pose_c22c1 = pose_c22w *pose_w2c1;

  Eigen::Vector4d point_c1;
  point_c1 << 0.5,0,0.2,1;
  
  Eigen::Vector4d point_c2 = pose_c22c1 * point_c1;
  std::cout<<"result:\n"<<point_c2.block<3,1>(0,0) <<std::endl;
    
  return 0;


  
}
