#include "Movable.h"

Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	speed = Eigen::Vector3f(0, 0, 0);
	isMoving = true;
}

void Movable::slide()
{
	this->MyTranslate(this->speed);
}

void Movable::setSpeed(Eigen::Vector3f speed)
{
	this->speed = speed;
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return T.matrix();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	T.pretranslate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	T.rotate(Eigen::AngleAxisf(angle, rotAxis));
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	T.scale(amt);
}

Eigen::Matrix3d Movable::GetRotation()
{
	return T.rotation().matrix().cast<double>();
}