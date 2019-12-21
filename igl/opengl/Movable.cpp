#include "Movable.h"


Movable::Movable()
{
	parent = nullptr;
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
}

void Movable::SetParent(Movable* view) {
	parent = view;
}

Eigen::Matrix4f Movable::MakeTrans()
{
	if (parent == nullptr) {
		return Tout.matrix();
	}
	return parent->MakeTrans() * Tout.matrix();
}

Eigen::Vector3f Movable::getTranslation() {
	return Tout.translation();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	/*if (preRotation)
		Tout.pretranslate(amt);
	else*/
	Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	Tout.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
}

//angle in radians
void Movable::MyRotate(Eigen::Matrix3f &rot, Eigen::Vector3f center)
{
	Tout.translate(-center);
	Tout.rotate(rot);
	Tout.translate(center);
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	Tout.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
	Tin.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation()
{
	return Tin.translation();
}