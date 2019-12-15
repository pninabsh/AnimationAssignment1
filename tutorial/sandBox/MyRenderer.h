#pragma once
#include "igl/opengl/glfw/renderer.h"
#include "MyViewer.h"
#include "IK_solver.h"

class MyRenderer : public Renderer {
private:
	MyViewer* my_viewer;
	void SceneMouseProcessing(int button);
	void ArmMouseMouseProcessing(int button);
public:
	void my_init(MyViewer* scn);
	MyViewer* GetMyScene();
	void MyMouseProcessing(int button);
};