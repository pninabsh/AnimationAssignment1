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

void MyRenderer::ArmMouseProcessing(int button) {
	int saved_index = my_viewer->selected_data_index;
	if (button == MOUSE_BUTTON_RIGHT) {
		for (int link_number : *my_viewer->links_numbers) {
			my_viewer->selected_data_index = link_number;
			MouseProcessing(button);
		}
	}
	else {
		for (int link_number : *my_viewer->links_numbers) {
			if (link_number <= saved_index) {
				my_viewer->selected_data_index = link_number;
				MouseProcessing(button);
			}
		}
	}

	my_viewer->selected_data_index = saved_index;
}

void MyRenderer::MyMouseProcessing(int button) {
	if (!my_viewer->is_object_selected) {
		SceneMouseProcessing(button);
	}
	else if (is_link(my_viewer->selected_data_index, *(my_viewer->links_numbers))) {
		ArmMouseProcessing(button);
	}
	else {
		MouseProcessing(button);
	}
}

void MyRenderer::BaseScale(double y) {
	my_viewer->data().MyTranslate(Eigen::Vector3f(0, 0, -y/10.0f));
}

void MyRenderer::SceneScale( double y) {
	int saved_index = my_viewer->selected_data_index;
	for (int i = 0; i < my_viewer->data_list.size(); i++) {
		my_viewer->selected_data_index = i;
		BaseScale(y);
	}
	my_viewer->selected_data_index = saved_index;
}

void MyRenderer::ArmScale(double y) {
	int saved_index = my_viewer->selected_data_index;
	for (int link_number : *my_viewer->links_numbers) {
		my_viewer->selected_data_index = link_number;
		BaseScale(y);
	}
	my_viewer->selected_data_index = saved_index;
}


void MyRenderer::MyScale(double y) {
	if (!my_viewer->is_object_selected) {
		SceneScale(y);
	}
	else if (is_link(my_viewer->selected_data_index, *(my_viewer->links_numbers))) {
		ArmScale(y);
	}
	else {
		BaseScale(y);
	}
}