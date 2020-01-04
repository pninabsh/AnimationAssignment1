#include "detectCollision.h"

Eigen::Vector4d calculate_point(Eigen::Vector3d point, igl::opengl::ViewerData collider) {
	Eigen::Vector4d point4d = Eigen::Vector4d(point(0), point(1), point(2), 1);
	return collider.MakeTrans().cast<double>() * point4d;
}

bool detect_x_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double x1 = calculate_point(bounding_box1.corner(bounding_box1.BottomLeft),collider1)(0);
	double x2 = calculate_point(bounding_box1.corner(bounding_box1.BottomRight), collider1)(0);
	
	double x3 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeft), collider2)(0);
	double x4 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeft), collider2)(0);

	return (x3 >= x1 && x3 <= x2) || (x4 >= x1 && x4 <= x2);
}

bool detect_y_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double y1 = calculate_point(bounding_box1.corner(bounding_box1.BottomLeft), collider1)(1);
	double y2 = calculate_point(bounding_box1.corner(bounding_box1.TopLeft), collider1)(1);

	double y3 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeft), collider2)(1);
	double y4 = calculate_point(bounding_box2.corner(bounding_box2.TopLeft), collider2)(1);

	return (y3 >= y1 && y3 <= y2) || (y4 >= y1 && y4 <= y2);
}

bool detect_z_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double z1 = calculate_point(bounding_box1.corner(bounding_box1.BottomLeftFloor), collider1)(2);
	double z2 = calculate_point(bounding_box1.corner(bounding_box1.BottomLeftCeil), collider1)(2);

	double z3 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeftFloor), collider2)(2);
	double z4 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeftCeil), collider2)(2);

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