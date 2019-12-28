#include "Movable.h"


Movable::Movable()
{
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	roty = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	rotx = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	roty2 = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	scale = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
};


Eigen::Vector3f Movable::getTranslation() {
	return Tout.translation();
}

Eigen::Matrix3f Movable::getRotation() {
	return roty.rotation() * rotx.rotation() * roty2.rotation();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	if (rotAxis == Eigen::Vector3f(1, 0, 0)) {
		rotx.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
	}
	else if (rotAxis == Eigen::Vector3f(0, 1, 0)) {
		roty.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
	}
	else {
		roty2.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
	}
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