#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "tutorial/sandBox/mesh_simplifier.h"

class MyViewer : public igl::opengl::glfw::Viewer {
	private:
		std::vector<SimplifyDataObject> *simplifyDataObjectsList;
		
	public:
		MyViewer() {
			simplifyDataObjectsList = new std::vector<SimplifyDataObject>();
		}
		~MyViewer() {
			delete simplifyDataObjectsList;
		}
		void load_configuration();
};