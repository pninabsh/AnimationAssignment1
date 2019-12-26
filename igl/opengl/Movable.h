#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
	// Tout * y * x * y2 * Tin * scale
public:
	Movable();
	void MyTranslate(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
	Eigen::Vector3f getTranslation();
	Eigen::Vector3f getCoordinates();
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	void SetParent(Movable* parent);
	Eigen::Matrix3f getRotation();
	Eigen::Transform<float,3,Eigen::Affine> Tin;
	Eigen::Transform<float, 3, Eigen::Affine> Tout;
	Eigen::Transform<float, 3, Eigen::Affine> roty;
	Eigen::Transform<float, 3, Eigen::Affine> rotx;
	Eigen::Transform<float, 3, Eigen::Affine> roty2;
	Eigen::Transform<float, 3, Eigen::Affine> scale;
};

