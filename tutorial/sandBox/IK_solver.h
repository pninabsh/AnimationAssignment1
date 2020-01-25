#pragma once
#include <vector>
#include "MyViewer.h"

extern int score;

extern igl::Timer timer;

bool is_link(int picked_object_index, std::vector<int> link_indices);

void start_IK_solver_animation();

void stop_IK_solver_animation();

void print_rotation_matrices(int picked_object_index, std::vector<int> link_indices, MyViewer* scn);

void print_arm_tip_positions(MyViewer* scn);

void print_destination_position(MyViewer* scn);

void rotate_y_axis(MyViewer* scn, int dir);

void rotate_x_axis(MyViewer* scn, int dir);

extern void ccd_step(igl::opengl::glfw::Viewer* scn, int hitObject);

bool getIsAnimating();