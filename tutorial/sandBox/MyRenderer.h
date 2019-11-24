#pragma once
#include "igl/opengl/glfw/renderer.h"
#include "MyViewer.h"

class MyRenderer : public Renderer {
private:
	MyViewer* my_viewer;
public:
	void my_init(MyViewer* scn);
	MyViewer* GetMyScene();
};