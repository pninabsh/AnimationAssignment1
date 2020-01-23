#include "DetectCollision.h"

Eigen::RowVector3d black(0, 0, 0);
Eigen::RowVector3d grey(0.5, 0.5, 0.5);
Eigen::RowVector3d blue(0, 0, 1);
Eigen::RowVector3d green(0, 1, 0);
Eigen::RowVector3d teal(0, 1, 1);
Eigen::RowVector3d red(1, 0, 0);
Eigen::RowVector3d pink(1, 0, 1);
Eigen::RowVector3d yellow(1, 1, 0);
Eigen::RowVector3d white(1, 1, 1);


Eigen::Vector3d calculate_point_vector3d(Eigen::Vector3d point, Eigen::Matrix4d make_trans) {
	Eigen::Vector4d point4d = Eigen::Vector4d(point(0), point(1), point(2), 1);
	Eigen::Vector4d point4d_scene = make_trans * point4d;
	return Eigen::Vector3d(point4d_scene(0), point4d_scene(1), point4d_scene(2));
}

bool check_collision_condition(double R0, double R1, double R) {
	return R <= R0 + R1;
}

bool detect_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2, Eigen::Matrix3d rot1, Eigen::Matrix3d rot2,
	Eigen::Vector3d A0, Eigen::Vector3d A1, Eigen::Vector3d A2, Eigen::Vector3d B0, Eigen::Vector3d B1, Eigen::Vector3d B2, Eigen::Matrix3d c,
	Eigen::Matrix4d make_trans_1, Eigen::Matrix4d make_trans2) {
	Eigen::Vector3d box1_center = calculate_point_vector3d(bounding_box1.center(), make_trans_1);
	Eigen::Vector3d box2_center = calculate_point_vector3d(bounding_box2.center(), make_trans2);
	Eigen::Vector3d d(3);
	d << box2_center(0) - box1_center(0), box2_center(1) - box1_center(1), box2_center(2) - box1_center(2);
	Eigen::Matrix3d positive_c = c.cwiseAbs();
	//calculate a0, a1, a2
	float a0 = bounding_box1.sizes()[0] / 2;
	float a1 = bounding_box1.sizes()[1] / 2;
	float a2 = bounding_box1.sizes()[2] / 2;
	//calculate b0,b1,b2
	float b0 = bounding_box2.sizes()[0] / 2;
	float b1 = bounding_box2.sizes()[1] / 2;
	float b2 = bounding_box2.sizes()[2] / 2;

	/*double a0_dot_d = A0.dot(d);
	double a1_dot_d = A1.dot(d);
	double a2_dot_d = A1.dot(d);
	double b0_dot_d = B0.dot(d);
	double b1_dot_d = B.dot(d);
	double b2_dot_d = B0.dot(d);*/

	double R0, R1, R;
	//1
	R0 = a0;
	R1 = b0 * positive_c(0, 0) + b1 * positive_c(0, 1) + b2 * positive_c(0, 2);
	R = std::abs(A0.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//2
	R0 = a1;
	R1 = b0 * positive_c(1, 0) + b1 * positive_c(1, 1) + b2 * positive_c(1, 2);
	R = std::abs(A1.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//3
	R0 = a2;
	R1 = b0 * positive_c(2, 0) + b1 * positive_c(2, 1) + b2 * positive_c(2, 2);
	R = std::abs(A2.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//4
	R0 = a0 * positive_c(0, 0) + a1 * positive_c(1, 0) + a2 * positive_c(2, 0);
	R1 = b0;
	R = std::abs(B0.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//5
	R0 = a0 * positive_c(0, 1) + a1 * positive_c(1, 1) + a2 * positive_c(2, 1);
	R1 = b1;
	R = std::abs(B1.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//6
	R0 = a0 * positive_c(0, 2) + a1 * positive_c(1, 2) + a2 * positive_c(2, 2);
	R1 = b2;
	R = std::abs(B2.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//7
	R0 = a1 * positive_c(2, 0) + a2 * positive_c(1, 0);
	R1 = b1 * positive_c(0, 2) + b2 * positive_c(0, 1);
	R = std::abs(c(1, 0) * A2.dot(d) - c(2, 0) * A1.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//8
	R0 = a1 * positive_c(2, 1) + a2 * positive_c(1, 1);
	R1 = b0 * positive_c(0, 2) + b2 * positive_c(0, 0);
	R = std::abs(c(1, 1) * A2.dot(d) - c(2, 1) * A1.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//9
	R0 = a1 * positive_c(2, 2) + a2 * positive_c(1, 2);
	R1 = b0 * positive_c(0, 1) + b1 * positive_c(0, 0);
	R = std::abs(c(1, 2) * A2.dot(d) - c(2, 2) * A1.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//10
	R0 = a0 * positive_c(2, 0) + a2 * positive_c(0, 0);
	R1 = b1 * positive_c(1, 2) + b2 * positive_c(1, 1);
	R = std::abs(c(2, 0) * A0.dot(d) - c(0, 0) * A2.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//11
	R0 = a0 * positive_c(2, 1) + a2 * positive_c(0, 1);
	R1 = b0 * positive_c(1, 2) + b2 * positive_c(1, 0);
	R = std::abs(c(2, 1) * A0.dot(d) - c(0, 1) * A2.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//12
	R0 = a0 * positive_c(2, 2) + a2 * positive_c(0, 2);
	R1 = b0 * positive_c(1, 1) + b1 * positive_c(1, 0);
	R = std::abs(c(2, 2) * A0.dot(d) - c(0, 2) * A2.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//13
	R0 = a0 * positive_c(1, 0) + a1 * positive_c(0, 0);
	R1 = b1 * positive_c(2, 2) + b2 * positive_c(2, 1);
	R = std::abs(c(0, 0) * A1.dot(d) - c(1, 0) * A0.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//14
	R0 = a0 * positive_c(1, 1) + a1 * positive_c(0, 1);
	R1 = b0 * positive_c(2, 2) + b2 * positive_c(2, 0);
	R = std::abs(c(0, 1) * A1.dot(d) - c(1, 1) * A0.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	//15
	R0 = a0 * positive_c(1, 2) + a1 * positive_c(0, 2);
	R1 = b0 * positive_c(2, 1) + b1 * positive_c(2, 0);
	R = std::abs(c(0, 2) * A1.dot(d) - c(1, 2) * A0.dot(d));
	if (R > R0 + R1) {
		return false;
	}

	return true;
	/*return
		check_collision_condition(a0, b0 * std::abs(c(0, 0)) + b1 * std::abs(c(0, 1)) + b2 * std::abs(c(0, 2)), std::abs(A0.dot(d)))
		&& check_collision_condition(a1, b0 * std::abs(c(1, 0)) + b1 * std::abs(c(1, 1)) + b2 * std::abs(c(1, 2)), std::abs(A1.dot(d)))
		&& check_collision_condition(a2, b0 * std::abs(c(2, 0)) + b1 * std::abs(c(2, 1)) + b2 * std::abs(c(2, 2)), std::abs(A2.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(0, 0)) + a1 * std::abs(c(1, 0)) + a2 * std::abs(c(2, 0)), b0, std::abs(B0.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(0, 1)) + a1 * std::abs(c(1, 1)) + a2 * std::abs(c(2, 1)), b1, std::abs(B1.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(0, 2)) + a1 * std::abs(c(1, 2)) + a2 * std::abs(c(2, 2)), b2, std::abs(B2.dot(d)))
		&& check_collision_condition(a1 * std::abs(c(2, 0)) + a2 * std::abs(c(1, 0)), b1 * std::abs(c(0, 2)) + b2 * std::abs(c(0, 1)), std::abs(c(1, 0) * A2.dot(d) - c(2, 0) * A1.dot(d)))
		&& check_collision_condition(a1 * std::abs(c(2, 1)) + a2 * std::abs(c(1, 1)), b0 * std::abs(c(0, 2)) + b2 * std::abs(c(0, 0)), std::abs(c(1, 1) * A2.dot(d) - c(2, 1) * A1.dot(d)))
		&& check_collision_condition(a1 * std::abs(c(2, 2)) + a2 * std::abs(c(1, 2)), b0 * std::abs(c(0, 1)) + b1 * std::abs(c(0, 0)), std::abs(c(1, 2) * A2.dot(d) - c(2, 2) * A1.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(2, 0)) + a2 * std::abs(c(0, 0)), b1 * std::abs(c(1, 2)) + b2 * std::abs(c(1, 1)), std::abs(c(2, 0) * A0.dot(d) - c(0, 0) * A2.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(2, 1)) + a2 * std::abs(c(0, 1)), b0 * std::abs(c(1, 2)) + b2 * std::abs(c(1, 0)), std::abs(c(2, 1) * A0.dot(d) - c(0, 1) * A2.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(2, 2)) + a2 * std::abs(c(0, 2)), b0 * std::abs(c(1, 1)) + b1 * std::abs(c(1, 0)), std::abs(c(2, 2) * A0.dot(d) - c(0, 2) * A2.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(1, 0)) + a1 * std::abs(c(0, 0)), b1 * std::abs(c(2, 2)) + b2 * std::abs(c(2, 1)), std::abs(c(0, 0) * A1.dot(d) - c(1, 0) * A0.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(1, 1)) + a1 * std::abs(c(0, 1)), b0 * std::abs(c(2, 2)) + b2 * std::abs(c(2, 0)), std::abs(c(0, 1) * A1.dot(d) - c(1, 1) * A0.dot(d)))
		&& check_collision_condition(a0 * std::abs(c(1, 2)) + a1 * std::abs(c(0, 2)), b0 * std::abs(c(2, 1)) + b1 * std::abs(c(2, 0)), std::abs(c(0, 2) * A1.dot(d) - c(1, 2) * A0.dot(d)));*/
}

bool find_collided_boxes_help(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1,
	igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2, Eigen::Matrix3d rot1, Eigen::Matrix3d rot2,
	Eigen::Vector3d A0, Eigen::Vector3d A1, Eigen::Vector3d A2, Eigen::Vector3d B0, Eigen::Vector3d B1, Eigen::Vector3d B2, Eigen::Matrix3d c,
	Eigen::Matrix4d make_trans_1, Eigen::Matrix4d make_trans_2) {
	bool collide = false;

	if (node1.is_leaf() && node2.is_leaf()) {
		return true;
	}

	else if (node1.is_leaf() && !node2.is_leaf()) {
		if (detect_collision(collider1, node1.m_box, collider2, (*node2.m_right).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, node1, collider2, *(node2.m_right), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, node1.m_box, collider2, (*node2.m_left).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, node1, collider2, *(node2.m_left), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
	}

	else if (!node1.is_leaf() && node2.is_leaf()) {
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, node2.m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_right), collider2, node2, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, node2.m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_left), collider2, node2, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
	}

	else {
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, (*node2.m_left).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_left), collider2, (*node2.m_left), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, (*node2.m_left).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_right), collider2, (*node2.m_left), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, (*node2.m_right).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_left), collider2, (*node2.m_right), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, (*node2.m_right).m_box, rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2)) {
			collide = find_collided_boxes_help(collider1, (*node1.m_right), collider2, (*node2.m_right), rot1, rot2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
			if (collide) {
				return true;
			}
		}

	}

	return false;
}

bool find_collided_boxes(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1,
	igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2) {
	Eigen::Matrix3d rotation1 = collider1.getRotation().cast<double>();
	Eigen::Matrix3d rotation2 = collider2.getRotation().cast<double>();
	Eigen::Vector3d A0 = rotation1 * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d A1 = rotation1 * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d A2 = rotation1 * Eigen::Vector3d(0, 0, 1);

	//calculate B0,B1,B2
	Eigen::Vector3d B0 = rotation2 * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d B1 = rotation2 * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d B2 = rotation2 * Eigen::Vector3d(0, 0, 1);

	Eigen::Matrix3d c = rotation1.inverse() * rotation2;
	Eigen::Matrix4d make_trans_1 = collider1.MakeTrans().cast<double>();
	Eigen::Matrix4d make_trans_2 = collider2.MakeTrans().cast<double>();
	return find_collided_boxes_help(collider1, node1, collider2, node2, rotation1, rotation2, A0, A1, A2, B0, B1, B2, c, make_trans_1, make_trans_2);
}