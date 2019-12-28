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
	if (button == 1) //right
	{

		my_viewer->MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0));
		my_viewer->MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0));
	}
	else //left
	{
		my_viewer->MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
		my_viewer->MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
	}
}

void MyRenderer::ArmMouseProcessing(int button) {
	int saved_index = my_viewer->selected_data_index;
	MouseProcessing(button);
	my_viewer->selected_data_index = saved_index;
}

void MyRenderer::MyMouseProcessing(int button) {
	if (is_link(my_viewer->selected_data_index, *(my_viewer->links_numbers))) {
		ArmMouseProcessing(button);
	}
	else if (!my_viewer->is_object_selected) {
		SceneMouseProcessing(button);
	}

	else {
		MouseProcessing(button);
	}
}

void MyRenderer::BaseScale(double y) {
	my_viewer->data().MyTranslate(Eigen::Vector3f(0, 0, -y/10.0f));
}

void MyRenderer::SceneScale( double y) {
	my_viewer->MyTranslate(Eigen::Vector3f(0, 0, -y / 10.0f));
}

void MyRenderer::ArmScale(double y) {
	int saved_index = my_viewer->selected_data_index;
	my_viewer->selected_data_index = 1;
	BaseScale(y);
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