#pragma once
#include "igl/opengl/glfw/renderer.h"
#include "MyViewer.h"
#include "IK_solver.h"

class MyRenderer : public Renderer {
private:
	MyViewer* my_viewer;
	
	void SceneMouseProcessing(int button);
	void ArmMouseProcessing(int button);

	void BaseScale(double y);
	void ArmScale(double y);
	void SceneScale(double y);
public:
	void my_init(MyViewer* scn);
	MyViewer* GetMyScene();
	void MyMouseProcessing(int button);
	void MyScale(double y);
};