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

void print_arm_tip_positions(MyViewer* scn) {
	/*Eigen::Matrix4f cylinderTopLoc = scn->data_list[1].MakeTrans();
	std::cout << "Arm top position: " << std::endl;
	std::cout << "x = " << cylinderTopLoc(0, 3) << " y = " << cylinderTopLoc(1, 3) << " z = " << cylinderTopLoc(2, 3) << std::endl;
	Eigen::Matrix4f cylinderButtomLoc = scn->data_list[4].MakeTrans();
	std::cout << "Arm buttom position: " << std::endl;
	std::cout << "x = " << cylinderButtomLoc(0, 3) << " y = " << cylinderButtomLoc(1, 3) << " z = " << cylinderButtomLoc(2, 3) << std::endl;*/
}

void print_destination_position(MyViewer* scn) {
	Eigen::Vector3f sphereLoc = scn->data_list[0].getTranslation();
	std::cout << "x = " << sphereLoc(0) << " y = " << sphereLoc(1) << " z = " << sphereLoc(2) << std::endl;
 }


void rotate_y_axis(MyViewer* scn) {
	Eigen::Matrix3f mul;
	Eigen::AngleAxisf rot_x = Eigen::AngleAxisf(0.0f, Eigen::Vector3f(1, 0, 0));
	Eigen::AngleAxisf rot_y = Eigen::AngleAxisf(5.0f, Eigen::Vector3f(0, 1, 0));
	mul = rot_y * rot_x * rot_y;
	int pressedIndex = scn->selected_data_index;
	scn->data_list[pressedIndex].MyRotate(Eigen::Vector3f(0, 1, 0), 0.5f);
}
void rotate_x_axis(MyViewer* scn) {
	// rotate x axis
	Eigen::Matrix3f mul;
	Eigen::AngleAxisf rot_x = Eigen::AngleAxisf(1.0f, Eigen::Vector3f(1, 0, 0));
	Eigen::AngleAxisf rot_y = Eigen::AngleAxisf(0.0f, Eigen::Vector3f(0, 1, 0));
	mul = rot_y * rot_x;
	int pressedIndex = scn->selected_data_index;
	scn->data_list[pressedIndex].MyRotate(Eigen::Vector3f(1, 0, 0), 0.5f);
}
