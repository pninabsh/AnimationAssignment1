#include "Movable.h"


Movable::Movable()
{
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	roty = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rot = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rotx = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	roty2 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	scale = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	speed = Eigen::Vector3f(0, 0, 0);
};

void Movable::slide()
{
	MyTranslate(speed);
}

Eigen::Vector3f Movable::getTranslation() {
	return Tout.translation();
}

void Movable::setSpeed(Eigen::Vector3f speed)
{
	this->speed = speed;
}

void Movable::reset() {
	Tin.translation().setZero();
	Tout.translation().setZero();
	roty.linear().setIdentity();
	rot.linear().setIdentity();
	rotx.linear().setIdentity();
	roty2.linear().setIdentity();
	speed = Eigen::Vector3f(0, 0, 0);
}

Eigen::Matrix3f Movable::getRotation() {
	return rot.rotation();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	rot.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
}

void Movable::MyRotate2(Eigen::AngleAxisf r)
{
	rot.rotate(r);
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	scale.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
	Tin.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation()
{
	return Tin.translation();
}