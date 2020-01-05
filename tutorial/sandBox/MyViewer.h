#pragma once
#include "igl/AABB.h"
#include "igl/opengl/glfw/Viewer.h"
#include "tutorial/sandBox/mesh_simplifier.h"
#include "detectCollision.h"

class MyViewer : public igl::opengl::glfw::Viewer {
	private:
		std::vector<SimplifyDataObject> *simplifyDataObjectsList;
		void create_bounding_box();
		
	public:
		MyViewer() {
			simplifyDataObjectsList = new std::vector<SimplifyDataObject>();
		}
		~MyViewer() {
			delete simplifyDataObjectsList;
		}
		void load_configuration();
};