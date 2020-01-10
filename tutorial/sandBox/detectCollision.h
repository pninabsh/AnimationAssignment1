#pragma once
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"

void draw_box(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color);
bool search_collided_boxes_in_trees(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1, std::vector< Eigen::AlignedBox<double, 3>>& collided_boxes_collider1,
	igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2, std::vector< Eigen::AlignedBox<double, 3>>& collided_boxes_collider2);
struct collided_boxes {
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider1;
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider2;
};

bool find_collided_boxes(igl::opengl::ViewerData &collider1, igl::opengl::ViewerData &collider2);