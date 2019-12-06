#include "MyRenderer.h"

void MyRenderer::my_init(MyViewer* scn) {
	my_viewer = scn;
}

MyViewer* MyRenderer::GetMyScene() {
	return my_viewer;
}