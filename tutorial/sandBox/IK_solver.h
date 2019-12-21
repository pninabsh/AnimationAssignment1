#pragma once
#include <vector>
#include "MyViewer.h"

bool is_link(int picked_object_index, std::vector<int> link_indices);

void toggle_IK_solver_animation();

void print_rotation_matrices(int picked_object_index, std::vector<int> link_indices);

void print_arm_tip_positions(MyViewer* scn);

void print_destination_position(MyViewer* scn);

void rotate_y_axis(MyViewer* scn);

void rotate_x_axis(MyViewer* scn);