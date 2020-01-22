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

void draw_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, std::vector<Eigen::AlignedBox3d::CornerType> vertices, Eigen::RowVector3d color) {
	Eigen::RowVector3d v1 = bounding_box.corner(vertices.at(0));
	Eigen::RowVector3d v2 = bounding_box.corner(vertices.at(1));
	Eigen::RowVector3d v3 = bounding_box.corner(vertices.at(2));
	Eigen::RowVector3d v4 = bounding_box.corner(vertices.at(3));
	mesh.add_edges(v1, v2, color);
	mesh.add_edges(v1, v3, color);
	mesh.add_edges(v2, v4, color);
	mesh.add_edges(v3, v4, color);
}

void draw_back_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopRightFloor,
		bounding_box.BottomLeftFloor, bounding_box.BottomRightFloor  }, color);
}

void draw_front_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftCeil, bounding_box.TopRightCeil,
		bounding_box.BottomLeftCeil, bounding_box.BottomRightCeil }, color);
}

void draw_top_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopRightFloor,
		bounding_box.TopLeftCeil, bounding_box.TopRightCeil  }, color);
}

void draw_bottom_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.BottomLeftFloor, bounding_box.BottomRightFloor,
		bounding_box.BottomLeftCeil, bounding_box.BottomRightCeil  }, color);
}

void draw_left_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopLeftCeil,
		bounding_box.BottomLeftFloor, bounding_box.BottomLeftCeil  }, color);
}

void draw_right_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopRightFloor, bounding_box.TopRightCeil,
		bounding_box.BottomRightFloor, bounding_box.BottomRightCeil  }, color);
}

void draw_box(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color) {
	draw_back_face(mesh, bounding_box, color);
	draw_front_face(mesh, bounding_box, color);
	draw_top_face(mesh, bounding_box, color);
	draw_bottom_face(mesh, bounding_box, color);
	draw_left_face(mesh, bounding_box, color);
	draw_right_face(mesh, bounding_box, color);
}

Eigen::Vector4d calculate_point(Eigen::Vector3d point, igl::opengl::ViewerData collider) {
	Eigen::Vector4d point4d = Eigen::Vector4d(point(0), point(1), point(2), 1);
	return collider.MakeTrans().cast<double>() * point4d;
}

Eigen::Vector3d calculate_point_vector3d(Eigen::Vector3d point, igl::opengl::ViewerData collider) {
	Eigen::Vector4d point4d = Eigen::Vector4d(point(0), point(1), point(2), 1);
	Eigen::Vector4d point4d_scene = collider.MakeTrans().cast<double>() * point4d;
	return Eigen::Vector3d(point4d_scene(0), point4d_scene(1), point4d_scene(2));
}

bool check_collision_condition(double R0, double R1, double R) {
	return R <= R0 + R1;
}

bool detect_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	Eigen::Matrix3d rotation1 = collider1.getRotation().cast<double>();
	Eigen::Matrix3d rotation2 = collider2.getRotation().cast<double>();
	Eigen::Vector3d box1_center = calculate_point_vector3d(bounding_box1.center(), collider1);
	Eigen::Vector3d box2_center = calculate_point_vector3d(bounding_box2.center(), collider2);
	Eigen::Vector3d d(3);
	d << box2_center(0) - box1_center(0), box2_center(1) - box1_center(1), box2_center(2) - box1_center(2);
	//calculate A0,A1,A2
	Eigen::Vector3d A0 = rotation1 * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d A1 = rotation1 * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d A2 = rotation1 * Eigen::Vector3d(0, 0, 1);

	//calculate B0,B1,B2
	Eigen::Vector3d B0 = rotation2 * Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d B1 = rotation2 * Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d B2 = rotation2 * Eigen::Vector3d(0, 0, 1);

	Eigen::Matrix3d c = rotation1.inverse() * rotation2;
	//calculate a0, a1, a2
	float a0 = bounding_box1.sizes()[0] / 2;
	float a1 = bounding_box1.sizes()[1] / 2;
	float a2 = bounding_box1.sizes()[2] / 2;
	//calculate b0,b1,b2
	float b0 = bounding_box2.sizes()[0] / 2;
	float b1 = bounding_box2.sizes()[1] / 2;
	float b2 = bounding_box2.sizes()[2] / 2;

	return
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
		&& check_collision_condition(a0 * std::abs(c(1, 2)) + a1 * std::abs(c(0, 2)), b0 * std::abs(c(2, 1)) + b1 * std::abs(c(2, 0)), std::abs(c(0, 2) * A1.dot(d) - c(1, 2) * A0.dot(d)));
}

bool find_collided_boxes(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1,
	igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2) {
	bool collide = false;

	if (node1.is_leaf() && node2.is_leaf()) {
		//draw_box(collider1, node1.m_box, white);
		//draw_box(collider2, node2.m_box, white);
		return true;
	}

	else if (node1.is_leaf() && !node2.is_leaf()) {
		if (detect_collision(collider1, node1.m_box, collider2, (*node2.m_right).m_box)) {
			collide = find_collided_boxes(collider1, node1, collider2, *(node2.m_right));
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, node1.m_box, collider2, (*node2.m_left).m_box)) {
			collide = find_collided_boxes(collider1, node1, collider2, *(node2.m_left));
			if (collide) {
				return true;
			}
		}
	}

	else if (!node1.is_leaf() && node2.is_leaf()) {
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, node2.m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_right), collider2, node2);
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, node2.m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_left), collider2, node2);
			if (collide) {
				return true;
			}
		}
	}

	else {
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, (*node2.m_left).m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_left), collider2, (*node2.m_left));
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, (*node2.m_left).m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_right), collider2, (*node2.m_left));
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_left).m_box, collider2, (*node2.m_right).m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_left), collider2, (*node2.m_right));
			if (collide) {
				return true;
			}
		}
		if (detect_collision(collider1, (*node1.m_right).m_box, collider2, (*node2.m_right).m_box)) {
			collide = find_collided_boxes(collider1, (*node1.m_right), collider2, (*node2.m_right));
			if (collide) {
				return true;
			}
		}

	}

	return false;
}