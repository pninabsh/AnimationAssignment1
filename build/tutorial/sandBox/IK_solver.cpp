#include "IK_solver.h"

static int isAnimating = false;

void start_IK_solver_animation() {
	// animate
}

void stop_IK_solver_animation(){
	// stop animation
}

void toggle_IK_solver_animation() {
	if (isAnimating) {
		stop_IK_solver_animation();
	}
	else {
		start_IK_solver_animation();
	}

	isAnimating = !isAnimating;
}

bool is_link(int picked_object_index, std::vector<int> link_indices) {
	for (int link_index : link_indices) {
		if (link_index == picked_object_index) {
			return true;
		}
	}

	return false;
}

void print_rotation_matrices(int picked_object_index, std::vector<int> link_indices) {
	if (!is_link(picked_object_index,link_indices)) {
		// print rotation of whole scene
	}

	// print rotation of the picked link
}

void print_arm_tip_positions() {
	// print arms tip positions
}

void print_destination_position() {
	// print destination position
 }


void rotate_y_axis() {
	// rotate y axis
}
void rotate_x_axis() {
	// rotate x axis
}
