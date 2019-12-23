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
	void MyRotate(Eigen::Matrix3f& rot, Eigen::Vector3f center);
	Eigen::Vector3f getTranslation();
	Eigen::Vector3f getCoordinates();
	void SetCenterOfRotation(Eigen::Vector3f amt);
	Eigen::Vector3f GetCenterOfRotation();
	void SetParent(Movable* parent);
	Eigen::Transform<float,3,Eigen::Affine> Tin;
	Eigen::Transform<float, 3, Eigen::Affine> Tout;
};

