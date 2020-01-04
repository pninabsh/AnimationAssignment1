#include "detectCollision.h"

bool detect_x_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	Eigen::RowVector3d oldX = bounding_box1.corner(bounding_box1.BottomLeft);
	Eigen::RowVector4d newX = Eigen::RowVector4d(oldX(0), oldX(1), oldX(2), 0);
	double X =  (newX * collider1.MakeTrans().cast<double>())(0);

	Eigen::RowVector3d oldX2 = bounding_box2.corner(bounding_box2.BottomLeft);
	Eigen::RowVector4d newX2 = Eigen::RowVector4d(oldX2(0), oldX2(1), oldX2(2), 1);
	double X2 = (newX2 * collider2.MakeTrans().cast<double>())(0);
	//double x2 = (collider1.MakeTrans() * bounding_box1.corner(bounding_box1.BottomRight))(0);
	
	//double x3 = (collider2.MakeTrans() * bounding_box2.corner(bounding_box2.BottomLeft))(0);
	//double x4 = (collider2.MakeTrans() * bounding_box2.corner(bounding_box2.BottomRight))(0);

	//return (x3 >= x1 && x3 <= x2) || (x4 >= x1 && x4 <= x2);
	return false;
}

bool detect_y_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double y1 = bounding_box1.corner(bounding_box1.BottomLeft)(1);
	double y2 = bounding_box1.corner(bounding_box1.TopLeft)(1);

	double y3 = bounding_box1.corner(bounding_box2.BottomLeft)(1);
	double y4 = bounding_box1.corner(bounding_box2.TopLeft)(1);

	return (y3 >= y1 && y3 <= y2) || (y4 >= y1 && y4 <= y2);
}

bool detect_z_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double z1 = bounding_box1.corner(bounding_box1.BottomLeftFloor)(2);
	double z2 = bounding_box1.corner(bounding_box1.BottomLeftCeil)(2);

	double z3 = bounding_box1.corner(bounding_box2.BottomLeftFloor)(2);
	double z4 = bounding_box1.corner(bounding_box2.BottomLeftCeil)(2);

	return (z3 >= z1&& z3 <= z2) || (z4 >= z1 && z4 <= z2);
}

//bool detect_collision(igl::AABB<Eigen::MatrixXd, 3> kd_tree1, igl::AABB<Eigen::MatrixXd, 3> kd_tree2) {
bool detect_collision(igl::opengl::ViewerData collider1, igl::opengl::ViewerData collider2) {
	Eigen::AlignedBox<double, 3> bounding_box1 = collider1.kd_tree.m_box;
	Eigen::AlignedBox<double, 3> bounding_box2 = collider2.kd_tree.m_box;

	return detect_x_collision(collider1, bounding_box1, collider2, bounding_box2)
		&& detect_y_collision(collider1, bounding_box1, collider2, bounding_box2)
		&& detect_z_collision(collider1, bounding_box1, collider2, bounding_box2);
}