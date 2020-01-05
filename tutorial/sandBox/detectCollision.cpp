#include "detectCollision.h"

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
	draw_back_face(mesh, bounding_box,color);
	draw_front_face(mesh, bounding_box,color);
	draw_top_face(mesh, bounding_box, color);
	draw_bottom_face(mesh, bounding_box, color);
	draw_left_face(mesh, bounding_box, color);
	draw_right_face(mesh, bounding_box, color);
}

Eigen::Vector4d calculate_point(Eigen::Vector3d point, igl::opengl::ViewerData collider) {
	Eigen::Vector4d point4d = Eigen::Vector4d(point(0), point(1), point(2), 1);
	return collider.MakeTrans().cast<double>() * point4d;
}

bool detect_x_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {
	double x1 = calculate_point(bounding_box1.corner(bounding_box1.BottomLeft),collider1)(0);
	double x2 = calculate_point(bounding_box1.corner(bounding_box1.BottomRight), collider1)(0);
	
	double x3 = calculate_point(bounding_box2.corner(bounding_box2.BottomLeft), collider2)(0);
	double x4 = calculate_point(bounding_box2.corner(bounding_box2.BottomRight), collider2)(0);

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

bool detect_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2) {

	return detect_x_collision(collider1, bounding_box1, collider2, bounding_box2)
		&& detect_y_collision(collider1, bounding_box1, collider2, bounding_box2)
		&& detect_z_collision(collider1, bounding_box1, collider2, bounding_box2);
}

void search_collided_boxes_in_trees(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1, std::vector< Eigen::AlignedBox<double, 3>>& collided_boxes_collider1,
	igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2, std::vector< Eigen::AlignedBox<double, 3>>& collided_boxes_collider2) {
	if(!detect_collision(collider1, node1.m_box,collider2,node2.m_box)){
		std::cout << "fail!" << std::endl;
		return;
	}

	if (!node1.is_leaf() && !node2.is_leaf()) {
		std::cout << "got here!" << std::endl;
		search_collided_boxes_in_trees(collider1, *(node1.m_left), collided_boxes_collider1, collider2, *(node2.m_left), collided_boxes_collider2);
		search_collided_boxes_in_trees(collider1, *(node1.m_left), collided_boxes_collider1, collider2, *(node2.m_right), collided_boxes_collider2);
		search_collided_boxes_in_trees(collider1, *(node1.m_right), collided_boxes_collider1, collider2, *(node2.m_left), collided_boxes_collider2);
		search_collided_boxes_in_trees(collider1, *(node1.m_right), collided_boxes_collider1, collider2, *(node2.m_right), collided_boxes_collider2);
		return;
	}

	if (node1.is_leaf()) {
		collided_boxes_collider1.push_back(node1.m_box);
		if (!node2.is_leaf()) {
			search_collided_boxes_in_trees(collider1, node1, collided_boxes_collider1, collider2, *(node2.m_left), collided_boxes_collider2);
			search_collided_boxes_in_trees(collider1, node1, collided_boxes_collider1, collider2, *(node2.m_right), collided_boxes_collider2);
		}
	}

	if (node2.is_leaf()) {
		collided_boxes_collider2.push_back(node2.m_box);
		if (!node1.is_leaf()) {
			search_collided_boxes_in_trees(collider1, *(node1.m_left), collided_boxes_collider1, collider2, node2, collided_boxes_collider2);
			search_collided_boxes_in_trees(collider1, *(node1.m_right), collided_boxes_collider1, collider2, node2, collided_boxes_collider2);
		}
	}
}

void find_collided_boxes(igl::opengl::ViewerData &collider1, igl::opengl::ViewerData &collider2) {

	collided_boxes collided_boxes_object;
	collided_boxes_object.collided_boxes_collider1 = std::vector< Eigen::AlignedBox<double, 3>>();
	collided_boxes_object.collided_boxes_collider2 = std::vector< Eigen::AlignedBox<double, 3>>();
	search_collided_boxes_in_trees(collider1, collider1.kd_tree, collided_boxes_object.collided_boxes_collider1,
		collider2, collider2.kd_tree, collided_boxes_object.collided_boxes_collider2);

	for (Eigen::AlignedBox<double, 3> box : collided_boxes_object.collided_boxes_collider1) {
		draw_box(collider1, box, green);
	}

	for (Eigen::AlignedBox<double, 3> box : collided_boxes_object.collided_boxes_collider2) {
		draw_box(collider2, box, blue);
	}

}