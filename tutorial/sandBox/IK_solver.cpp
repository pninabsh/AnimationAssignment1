#include "IK_solver.h"
#include <igl/opengl/glfw/Viewer.h>
using namespace std;

bool isAnimating = false;
igl::Timer timer;
int score = 0;

Eigen::Vector3d getCoordinates(igl::opengl::ViewerData link, bool upLink) {
	Eigen::Vector3d mLink = link.V.colwise().minCoeff();
	Eigen::Vector3d MLink = link.V.colwise().maxCoeff();
	Eigen::Vector4f helpingVector(4);
	if (upLink) {
		helpingVector << (MLink(0) + mLink(0)) / 2, MLink(1), (MLink(2) + mLink(2)) / 2, 1;
	}
	else {
		helpingVector << (MLink(0) + mLink(0)) / 2, mLink(1), (MLink(2) + mLink(2)) / 2, 1;
	}
	Eigen::Vector4f mulVector = link.MakeTrans() * helpingVector;
	return Eigen::Vector3d(mulVector(0), mulVector(1), mulVector(2));
}

float distance(Eigen::Vector3f p1, Eigen::Vector3d p2) {
	float res = sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2) + pow(p1(2) - p2(2), 2));
	return res;
}

void ccd_step(MyViewer* scn, int hitObject) {
	//d and e are the same in all iterations
	//changes between iterations
	bool collide = find_collided_boxes(scn->data_list[hitObject], scn->data_list[hitObject].kd_tree, scn->data_list[14], scn->data_list[14].kd_tree);
	if (!collide) {
		for (int i = 14; i >= 5; i--) {
			Eigen::Vector3f d = scn->data_list[hitObject].getTranslation();
			Eigen::Vector3d e = getCoordinates(scn->data_list[14], true);
			Eigen::Vector3d r = getCoordinates(scn->data_list[i], false);
			Eigen::Vector3d re = (e - r).normalized();
			Eigen::Vector3d rd;
			rd << d(0) - r(0), d(1) - r(1), d(2) - r(2);
			rd = rd.normalized();
			auto plane = re.cross(rd);
			Eigen::Vector3f planeVector(plane(0), plane(1), plane(2));
			auto dotProduct = re.dot(rd);
			if (dotProduct < -1 || dotProduct > 1) {
				std::cout << "The angle is not between -1 and 1" << std::endl;
				return;
			}
			auto theta = acos(dotProduct);
			theta = theta / 5;
			scn->data_list[i].MyRotate(planeVector, theta);
			e = getCoordinates(scn->data_list[14], true);
			bool collide = find_collided_boxes(scn->data_list[hitObject], scn->data_list[hitObject].kd_tree, scn->data_list[14], scn->data_list[14].kd_tree);
			if (collide) {
				return;
			}
		}
	}
	else if(isAnimating) {
		isAnimating = false;
		scn->data_list[hitObject].set_visible(false);
		score += 20;
		scn->sound_manager.play_hit();
		std::cout << "The score is: " << score << std::endl;
		return;
	}
}

void start_IK_solver_animation() {
	isAnimating = true;
	//ccd_step(scn, pickedObject);
}

bool getIsAnimating() {
	return isAnimating;
}

void stop_IK_solver_animation() {
	isAnimating = false;
}

bool is_link(int picked_object_index, std::vector<int> link_indices) {
	for (int link_index : link_indices) {
		if (link_index == picked_object_index) {
			return true;
		}
	}

	return false;
}

void print_rotation_matrices_helping(Eigen::Matrix3f rot) {
	std::cout << "The rotation matrix is: " << std::endl;
	std::cout << rot(0, 0) << rot(0, 1) << rot(0, 2) << std::endl;
	std::cout << rot(1, 0) << rot(1, 1) << rot(1, 2) << std::endl;
	std::cout << rot(2, 0) << rot(2, 1) << rot(2, 2) << std::endl;
}

void print_rotation_matrices(int picked_object_index, std::vector<int> link_indices, MyViewer* scn) {
	Eigen::Matrix3f rotMat;
	if (!is_link(picked_object_index, link_indices) || !scn->is_object_selected) {
		// print rotation of whole scene
		rotMat = scn->getRotation();
	}
	else {
		// print rotation of the picked link
		rotMat = scn->data_list[picked_object_index].getRotation();
	}
	print_rotation_matrices_helping(rotMat);
}

void print_arm_tip_positions(MyViewer* scn) {
	Eigen::Vector3d topPoint = getCoordinates(scn->data_list[4], true);
	std::cout << "Top arm tip: " << std::endl;
	std::cout << "x = " << topPoint(0) << " y = " << topPoint(1) << " z = " << topPoint(2) << std::endl;
}

void print_destination_position(MyViewer* scn) {
	Eigen::Vector3f sphereLoc = scn->data_list[0].getTranslation();
	std::cout << "x = " << sphereLoc(0) << " y = " << sphereLoc(1) << " z = " << sphereLoc(2) << std::endl;
}


void rotate_y_axis(MyViewer* scn, int dir) {
	if (!scn->is_object_selected) {
		auto m = scn->getRotation().transpose();
		scn->MyRotate(m * Eigen::Vector3f(0, 1, 0), dir * 0.1f);
	}
	else {
		int pressedIndex = scn->selected_data_index;
		auto m = scn->data_list[pressedIndex].getRotation().transpose();
		scn->data_list[pressedIndex].MyRotate(m * Eigen::Vector3f(0, 1, 0), dir * 0.1f);
	}
}

void rotate_x_axis(MyViewer* scn, int dir) {
	if (!scn->is_object_selected) {
		auto m = scn->getRotation().transpose();
		scn->MyRotate(m * Eigen::Vector3f(1, 0, 0), dir * 0.1f);
	}
	else {
		int pressedIndex = scn->selected_data_index;
		auto m = scn->data_list[pressedIndex].getRotation().transpose();
		scn->data_list[pressedIndex].MyRotate(m * Eigen::Vector3f(1, 0, 0), dir * 0.1f);
	}
}
