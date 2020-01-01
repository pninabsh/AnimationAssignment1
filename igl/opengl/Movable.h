#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	Eigen::Matrix4f MakeTrans();
	void slide();
	void setSpeed(Eigen::Vector3f speed);
	void MyTranslate(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
private:
	Eigen::Transform<float,3,Eigen::Affine> T;
	Eigen::Vector3f speed;

};

