#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "tutorial/sandBox/mesh_simplifier.h"

class MyViewer : public igl::opengl::glfw::Viewer {
	private:
		std::vector<SimplifyDataObject> simplifyDataObjectsList;

	public:
		void load_configuration();
		void init_simplify_data_structures_list();
		void simplify();
};