#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();
	void reset();
	void MyTranslate(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
	Eigen::Vector3f getTranslation();
	Eigen::Vector3f getCoordinates();
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	void setSpeed(Eigen::Vector3f speed);
	void SetParent(Movable* parent);
	void slide();
	Eigen::Matrix3f getRotation();
	Eigen::Transform<float,3,Eigen::Affine> Tin;
	Eigen::Transform<float, 3, Eigen::Affine> Tout;
	Eigen::Transform<float, 3, Eigen::Affine> rot;
	Eigen::Transform<float, 3, Eigen::Affine> roty;
	Eigen::Transform<float, 3, Eigen::Affine> rotx;
	Eigen::Transform<float, 3, Eigen::Affine> roty2;
	Eigen::Transform<float, 3, Eigen::Affine> scale;
	Eigen::Vector3f speed;
};

