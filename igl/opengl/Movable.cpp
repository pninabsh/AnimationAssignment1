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

void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	if (rotAxis == Eigen::Vector3f(1, 0, 0)) {
		//Tout.translate(Eigen::Vector3f(0, 0.5, 0));
		rotx.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
		//Tout.translate(Eigen::Vector3f(0, -0.5, 0));
	}
	else {
		
		roty.rotate(Eigen::AngleAxisf(angle, rotAxis.normalized()));
	}
}

void Movable::MyScale(Eigen::Vector3f amt)
{
	scale.scale(amt);
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