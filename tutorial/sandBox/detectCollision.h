#pragma once
#include "igl/opengl/ViewerData.h"
#include "igl/AABB.h"

void draw_box(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, Eigen::RowVector3d color);


bool detect_collision(igl::opengl::ViewerData collider1, Eigen::AlignedBox<double, 3> bounding_box1,
	igl::opengl::ViewerData collider2, Eigen::AlignedBox<double, 3> bounding_box2);

struct collided_boxes {
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider1;
	std::vector<Eigen::AlignedBox<double, 3>> collided_boxes_collider2;
};

void find_collided_boxes(igl::opengl::ViewerData &collider1, igl::opengl::ViewerData &collider2);