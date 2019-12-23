#include "Movable.h"


Movable::Movable()
{
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();

}

Eigen::Vector3f Movable::getTranslation() {
	return Tout.translation();
}

void Movable::MyTranslate(Eigen::Vector3f amt)
{
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

/*Eigen::Vector3f getCoordinates() {
	Eigen::Vector3d res(3);
	Eigen::Vector3d trans = Tout.translation();
	res << trans(0, 3), trans(1, 3), trans(2, 3);
	return res;
}*/

void Movable::SetCenterOfRotation(Eigen::Vector3f amt)
{
	Tin.translate(amt);
}

Eigen::Vector3f Movable::GetCenterOfRotation()
{
	return Tin.translation();
}