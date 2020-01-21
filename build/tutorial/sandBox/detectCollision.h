#pragma once
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"

bool find_collided_boxes(igl::opengl::ViewerData collider1, igl::AABB<Eigen::MatrixXd, 3> node1, igl::opengl::ViewerData collider2, igl::AABB<Eigen::MatrixXd, 3> node2);
struct collided_boxes {
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider1;
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider2;
};