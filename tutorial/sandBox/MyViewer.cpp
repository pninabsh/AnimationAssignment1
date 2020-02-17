#include "MyViewer.h"
#include <iostream>
#include <fstream>
#include "IK_solver.h"
#include <tutorial\sandBox\MyRenderer.h>
#include <tutorial\sandBox\inputManager.h>
#include <igl/png/readPNG.h>


using namespace igl::png;
using namespace igl;
using namespace std;

Display* disp = NULL;
MyRenderer renderer;
Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;

void MyViewer::create_bounding_box() {
	igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(this->data_list[0].V, this->data_list[0].F);
	init_simplify_data_structures_list();

	for (int i = 0; i < simplifyDataObjectsList->size(); i++) {
		tree.init(simplifyDataObjectsList->at(i).V, simplifyDataObjectsList->at(i).F);
		this->data_list[i].kd_tree = tree;
	}
}

void fail_load_resource(string& sphere_path, string& yCylinder_path, string& cube_path,
	string& catch_sound_path, string& hit_sound_path, string& end_level_sound_path)
{
	cout << "cannot open or read all needed line from configuration file! loading from default mesh paths instead..." << endl;
	sphere_path = "C:/Dev/EngineIGLnew/tutorial/data/sphere.obj";
	yCylinder_path = "C:/Dev/EngineIGLnew/tutorial/data/ycylinder.obj";
	cube_path = "C:/Dev/EngineIGLnew/tutorial/data/cube.obj";
	catch_sound_path = "C:/Dev/EngineIGLnew/tutorial/data/catch.wav";
	hit_sound_path = "C:/Dev/EngineIGLnew/tutorial/data/hit.wav";
	end_level_sound_path = "C:/Dev/EngineIGLnew/tutorial/data/end_level.wav";
}

bool attempt_load_resource(string& mesh_path, ifstream& configuration_file)
{
	if (configuration_file.good())
	{
		getline(configuration_file, mesh_path);
		return true;
	}
	return false;
}


void MyViewer::load_configuration()
{
	string sphere_path;
	string yCylinder_path;
	string cube_path;
	string catch_sound_path;
	string hit_sound_path;
	string end_level_sound_path;
	cout << "reading mesh paths from configuration.txt file..." << endl;

	ifstream configuration_file("configuration.txt");

	if (configuration_file.fail())
	{
		fail_load_resource(sphere_path, yCylinder_path, cube_path, catch_sound_path, hit_sound_path, end_level_sound_path);
	}
	else
	{
		cout << "loading resources from configuration file..." << endl;
		bool isAllResourcesLoaded = attempt_load_resource(sphere_path, configuration_file) && attempt_load_resource(yCylinder_path, configuration_file)
			&& attempt_load_resource(cube_path, configuration_file)
			&& attempt_load_resource(catch_sound_path, configuration_file) && attempt_load_resource(hit_sound_path, configuration_file)
			&& attempt_load_resource(end_level_sound_path, configuration_file);

		if (!isAllResourcesLoaded)
		{
			fail_load_resource(sphere_path, yCylinder_path, cube_path, catch_sound_path, hit_sound_path, end_level_sound_path);
		}
	}

	this->sound_manager.configureSoundPaths(catch_sound_path, hit_sound_path, end_level_sound_path);
	//readPNG("snaketexture2.jpg", R, G, B, A);
	for (int i = 0; i < 5; i++) {
		load_mesh_from_file(sphere_path);
	}

	for (int i = 0; i < 10; i++) {
		load_mesh_from_file(yCylinder_path);
	}
	cout << "loading done!" << endl;
	configuration_file.close();
}
void MyViewer::init_simplify_data_structures_list()
{
	simplifyDataObjectsList->clear();
	for (igl::opengl::ViewerData viewer_data : data_list)
	{
		simplifyDataObjectsList->push_back(get_SimplifyDataObject(viewer_data));
	}
}

void MyViewer::simplify()
{
	SimplifyDataObject selectedSimplifyDataObject;

	const auto do_simplify = [this, &selectedSimplifyDataObject](double number_of_edges) -> void {
		std::vector<PriorityQueue::iterator> Qit;
		Qit.resize(selectedSimplifyDataObject.E.rows());

		for (int i = 0; i < number_of_edges; i++)
		{
			for (auto it = selectedSimplifyDataObject.Q.begin(); it != selectedSimplifyDataObject.Q.end(); ++it)
			{
				Qit[it->second] = it;
			};

			if (!collapse_edge(selectedSimplifyDataObject, Qit))
			{
				break;
			}
		}

		simplifyDataObjectsList->at(selected_data_index) = selectedSimplifyDataObject;
	};

	selectedSimplifyDataObject = simplifyDataObjectsList->at(selected_data_index);
	double rounded_up_five_percent_edges = std::ceil(0.8 * selectedSimplifyDataObject.Q.size());
	do_simplify(rounded_up_five_percent_edges);
}

void MyViewer::organize_spheres_on_board1()
{
	data_list[0].MyTranslate(Eigen::Vector3f(2, -6, 0));
	data_list[0].setSpeed(Eigen::Vector3f(0, 0.03f, 0));

	data_list[1].MyTranslate(Eigen::Vector3f(-2, -6, 0));
	data_list[1].setSpeed(Eigen::Vector3f(0, 0.01f, 0));

	data_list[2].MyTranslate(Eigen::Vector3f(-6, -6, 0));
	data_list[2].setSpeed(Eigen::Vector3f(0.05f, 0, 0));

	data_list[3].MyTranslate(Eigen::Vector3f(6, 6, 0));
	data_list[3].setSpeed(Eigen::Vector3f(0, -0.01f, 0));

	data_list[4].MyTranslate(Eigen::Vector3f(-9, 5, 0));
	data_list[4].setSpeed(Eigen::Vector3f(0.07f, 0, 0));
	for (int i = 0; i < 5; i++) {
		data_list[i].set_visible(true);
	}
}

void MyViewer::organize_spheres_on_board2()
{
	data_list[0].MyTranslate(Eigen::Vector3f(1, 3, 0));
	data_list[0].setSpeed(Eigen::Vector3f(0, 0.02f, 0));

	data_list[1].MyTranslate(Eigen::Vector3f(2, 4, 0));
	data_list[1].setSpeed(Eigen::Vector3f(0, 0.04f, 0));

	data_list[2].MyTranslate(Eigen::Vector3f(-4, -4, 0));
	data_list[2].setSpeed(Eigen::Vector3f(0.03f, 0, 0));

	data_list[3].MyTranslate(Eigen::Vector3f(-3, -3, 0));
	data_list[3].setSpeed(Eigen::Vector3f(0, 0.005f, 0));

	data_list[4].MyTranslate(Eigen::Vector3f(-5, 5, 0));
	data_list[4].setSpeed(Eigen::Vector3f(0.09f, 0, 0));
	for (int i = 0; i < 5; i++) {
		data_list[i].set_visible(true);
	}
}

void MyViewer::organize_snake() {
	links_numbers->clear();
	parent_links_indices->clear();
	//label_color = 


	for (int i = 5; i <= 14; i++) {
		data_list[i].show_texture = true;
		data_list[i].set_texture(R, G, B);
	}

	for (int i = 6; i <= 14; i++) {
		data_list[i].SetParent(&(data_list[i - 1]));
	}

	data_list[5].SetCenterOfRotation(Eigen::Vector3f(0, -2, 0));
	data_list[5].MyTranslate(Eigen::Vector3f(0, -6, 0));
	for (int i = 6; i <= 14; i++) {
		data_list[i].SetCenterOfRotation(Eigen::Vector3f(0, 0.8, 0));
	}
	for (int i = 6; i <= 14; i++) {
		data_list[i].MyTranslate(Eigen::Vector3f(0, 0.8, 0));
	}
	for (int i = 5; i < data_list.size(); i++) {
		links_numbers->push_back(i);

		if (i != 14) {
			parent_links_indices->push_back(links_numbers->at(links_numbers->size() - 1));
		}
	}

	Eigen::RowVector3d color(1, 0, 0);
	data_list[5].set_colors(color);
	/*Eigen::Vector3d m = data_list[5].V.colwise().minCoeff();
	Eigen::Vector3d M = data_list[5].V.colwise().maxCoeff();
	std::stringstream l1;
	l1 << m(0) << ", " << m(1) << ", " << m(2);
	data_list[5].add_label(m, l1.str());
	std::stringstream l2;
	l2 << M(0) << ", " << M(1) << ", " << M(2);*/
	//data().add_label(Eigen::Vector3d(1,0,0), "hahahahaha");

	//igl::opengl::glfw::imgui::ImGuiMenu menu; 
	//menu.init()
	//menu.callback_draw_viewer_window = []() {};
	/*menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		menu.draw_viewer_menu();
		double doubleVariable = 0.1f; // Shared between two menus
		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
			ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable))
			{
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int*)(&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices))
			{
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int)choices.size())
			{
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
			{
				std::cout << "Hello\n";
			}
		}
	};*/

	//plugins.push_back(&menu);
	//data().add_points(, Eigen::RowVector3d(1, 0, 0));
	//data().add_label(Eigen::Vector3d(5, 2, -2), "Score: ");
}

void  MyViewer::create_level() {
	if (level % 2 == 1) {
		organize_spheres_on_board1();
	}
	else {
		organize_spheres_on_board2();
	}
	organize_snake();
	create_bounding_box();
	timer.start();

}

void MyViewer::level_reset(bool to_level_up) {
	if (waiting_for_user_answer) {
		waiting_for_user_answer = false;

		if (to_level_up) {
			level++;
		}

		for (int i = 0; i < data_list.size(); i++) {
			data_list[i].reset();
		}
		renderer.core(2).background_color << 0.3f, 0.3f, 0.5f, 1.0f;
		renderer.core(1).background_color << 0.3f, 0.3f, 0.5f, 1.0f;
		load_configuration();
		create_level();
	}
}

void MyViewer::end_level() {
	stop_IK_solver_animation();
	timer.stop();
	sound_manager.play_end_level();
	std::cout << "Level number: " << level << std::endl;
	std::cout << "The final score you achieved is: " << score << std::endl;
	std::cout << "Press enter for replaying the level, or press space to continue to the next level" << std::endl;
	waiting_for_user_answer = true;
}

bool MyViewer::is_waiting_for_user() {
	return waiting_for_user_answer;
}

void MyViewer::Initialize_scene() {
	if (disp == NULL) {
		disp = new Display(1200, 1000, "Welcome");
	}
	//igl::opengl::glfw::imgui::ImGuiMenu menu;
	//float floatVariable;
	/*menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		menu.draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
			ImGui::InputFloat("float", &floatVariable, 0, 0, 3);

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable))
			{
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int*)(&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices))
			{
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int)choices.size())
			{
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
			{
				std::cout << "Hello\n";
			}
		}
	};*/
	// Draw additional windows
	/*menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
		ImGui::Begin(
			"New Window", nullptr,
			ImGuiWindowFlags_NoSavedSettings
		);

		// Expose the same variable directly ...
		ImGui::PushItemWidth(-80);
		ImGui::DragFloat("float", &floatVariable, 0.0, 0.0, 3.0);
		ImGui::PopItemWidth();

		static std::string str = "bunny";
		ImGui::InputText("Name", str);

		ImGui::End();
	};*/
	// Customize the menu
	//double doubleVariable = 0.1f; // Shared between two menus
	// Add content to the default menu window

	bool show_demo_window = true;
	load_configuration();
	create_level();
	init_simplify_data_structures_list();
	Init(*disp);
	renderer.callback_post_resize = [&](int w, int h) {
		renderer.core(1).viewport = Eigen::Vector4f(0, 0, w / 2, h);
		renderer.core(2).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
		return true;
	};
	renderer.init(this);
	renderer.my_init(this);
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);
	//delete disp;
}

