#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "tutorial/sandBox/mesh_simplifier.h"

class MyViewer : public igl::opengl::glfw::Viewer {
	private:
		std::vector<SimplifyDataObject> *simplifyDataObjectsList;

		float resize_value = 0.3;
		float arm_part_position = 0.48;
		void setup_arm_link_midpoint(igl::opengl::ViewerData &link);
		void setup_arm_link_axis(igl::opengl::ViewerData &link, int previous_link_index);
		
	public:
		bool is_object_selected;
		std::vector<int>* parent_links_indices;
		std::vector<int>* links_numbers;
		MyViewer() {
			simplifyDataObjectsList = new std::vector<SimplifyDataObject>();
			parent_links_indices = new std::vector<int>();
			links_numbers = new std::vector<int>();
		}
		~MyViewer() {
			delete simplifyDataObjectsList;
			delete parent_links_indices;
			delete links_numbers;
		}
		void load_configuration();
		void load_configuration_IK();
		void init_simplify_data_structures_list();
		void simplify();
};