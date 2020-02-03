#pragma once
#include "igl/AABB.h"
#include <igl/Timer.h>
#include "igl/opengl/glfw/Viewer.h"
#include "tutorial/sandBox/mesh_simplifier.h"
#include "tutorial/sandBox/DetectCollision.h"
#include "tutorial/sandBox/SoundManager.h"


class MyViewer : public igl::opengl::glfw::Viewer {
private:
	int level;
	bool waiting_for_user_answer;
	std::vector<SimplifyDataObject>* simplifyDataObjectsList;

	float resize_value = 1;
	float arm_part_position = 0.48;
	void create_bounding_box();

public:
	SoundManager sound_manager;
	bool is_object_selected;
	std::vector<int>* parent_links_indices;
	std::vector<int>* links_numbers;
	void Initialize_scene();
	MyViewer() {
		level = 1;
		waiting_for_user_answer = false;
		simplifyDataObjectsList = new std::vector<SimplifyDataObject>();
		parent_links_indices = new std::vector<int>();
		links_numbers = new std::vector<int>();
		sound_manager = SoundManager();
	}
	~MyViewer() {
		delete simplifyDataObjectsList;
		delete parent_links_indices;
		delete links_numbers;
	}
	void create_level();
	void load_configuration();
	void init_simplify_data_structures_list();
	void simplify();
	void organize_spheres_on_board1();
	void organize_spheres_on_board2();
	void organize_snake();
	void end_level();
	void level_reset(bool to_level_up);
	bool is_waiting_for_user();
};