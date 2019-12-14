#pragma once
#include <vector>

void toggle_IK_solver_animation();

void print_rotation_matrices(int picked_object_index, std::vector<int> link_indices);

void print_arm_tip_positions();

void print_destination_position();

void rotate_y_axis();

void rotate_x_axis();