#include "MyRenderer.h"

int MOUSE_BUTTON_LEFT = 0;
int MOUSE_BUTTON_RIGHT = 1;

void MyRenderer::my_init(MyViewer* scn) {
	my_viewer = scn;
}

MyViewer* MyRenderer::GetMyScene() {
	return my_viewer;
}

void MyRenderer::SceneMouseProcessing(int button) {
	int saved_index = my_viewer->selected_data_index;
	for(int i=0;i<my_viewer->data_list.size();i++){
		my_viewer->selected_data_index = i;
		MouseProcessing(button);
	}
	my_viewer->selected_data_index = saved_index;
}

void MyRenderer::ArmMouseMouseProcessing(int button) {
	if (MOUSE_BUTTON_RIGHT) {
		int saved_index = my_viewer->selected_data_index;
		for (int link_number : *my_viewer->links_numbers) {
			my_viewer->selected_data_index = link_number;
			MouseProcessing(button);
		}
		my_viewer->selected_data_index = saved_index;
	}
	else {

	}
}

void MyRenderer::MyMouseProcessing(int button) {
	if (!my_viewer->is_object_selected) {
		SceneMouseProcessing(button);
	}
	else if (is_link(my_viewer->selected_data_index, *(my_viewer->links_numbers))) {
		ArmMouseMouseProcessing(button);
	}
	else {
		MouseProcessing(button);
	}
}

void MyRenderer::BaseScale(double x, double y) {
	my_viewer->data().MyScale(Eigen::Vector3f(1 + y * 0.01, 1 + y * 0.01, 1 + y * 0.01));
}

void MyRenderer::SceneScale(double x, double y) {
	int saved_index = my_viewer->selected_data_index;
	for (int i = 0; i < my_viewer->data_list.size(); i++) {
		my_viewer->selected_data_index = i;
		BaseScale(x,y);
	}
	my_viewer->selected_data_index = saved_index;
}

void MyRenderer::ArmScale(double x, double y) {
	int saved_index = my_viewer->selected_data_index;
	for (int link_number : *my_viewer->links_numbers) {
		my_viewer->selected_data_index = link_number;
		BaseScale(x,y);
	}
	my_viewer->selected_data_index = saved_index;
}


void MyRenderer::MyScale(double x, double y) {
	if (!my_viewer->is_object_selected) {
		SceneScale(x, y);
	}
	else if (is_link(my_viewer->selected_data_index, *(my_viewer->links_numbers))) {
		ArmScale(x, y);
	}
	else {
		BaseScale(x,y);
	}
}