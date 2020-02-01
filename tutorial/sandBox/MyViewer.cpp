#include "MyViewer.h"
#include <iostream>
#include <fstream>
#include "IK_solver.h"
#include <tutorial\sandBox\MyRenderer.h>
#include <tutorial\sandBox\inputManager.h>
using namespace igl;
using namespace std;

int level = 0;
Display* disp = NULL;

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
		fail_load_resource(sphere_path, yCylinder_path, cube_path,catch_sound_path,hit_sound_path, end_level_sound_path);
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

	for (int i = 0; i < 5; i++) {
		load_mesh_from_file(sphere_path);
	}

	for (int i = 0; i < 10; i++) {
		load_mesh_from_file(yCylinder_path);
	}

	for (int i = 6; i <= 14; i++) {
		data_list[i].SetParent(&(data_list[i - 1]));
	}

	/*data_list[0].MyTranslate(Eigen::Vector3f(2, -6, 0));
	data_list[0].setSpeed(Eigen::Vector3f (0, 0.01f, 0));*/
	organize_spheres_on_board();
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
	cout << "loading done!" << endl;
	configuration_file.close();
	create_bounding_box();
	timer.start();
}
void MyViewer::init_simplify_data_structures_list()
{
	for (igl::opengl::ViewerData viewer_data : data_list)
	{
		simplifyDataObjectsList->push_back(get_SimplifyDataObject(viewer_data));
	}
}

void MyViewer::simplify()
{
	SimplifyDataObject selectedSimplifyDataObject;

	const auto do_simplify = [this, &selectedSimplifyDataObject](double number_of_edges) -> void {
		//bool something_collapsed = false;

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
			//something_collapsed = true;
			//update_v_planes(selectedSimplifyDataObject);
			//update_q_matrixes(selectedSimplifyDataObject);
			//update_priority_queue(selectedSimplifyDataObject);
		}

		simplifyDataObjectsList->at(selected_data_index) = selectedSimplifyDataObject;

		/*if (something_collapsed)
		{
			data().clear();
			data().set_mesh(selectedSimplifyDataObject.V, selectedSimplifyDataObject.F);
			data().set_face_based(true);
			// use the modified V and F and F_NORMALS to re-calculate E,EF,EI,Q,C,EMAP,V_Q_MATRIX,V_PLANES
			get_SimplifyDataObject(selectedSimplifyDataObject);
		}*/
	};

	selectedSimplifyDataObject = simplifyDataObjectsList->at(selected_data_index);
	double rounded_up_five_percent_edges = std::ceil(0.8 * selectedSimplifyDataObject.Q.size());
	do_simplify(rounded_up_five_percent_edges);
}

void MyViewer::organize_spheres_on_board()
{
	data_list[0].MyTranslate(Eigen::Vector3f(2, -6, 0));
	data_list[0].setSpeed(Eigen::Vector3f(0, 0.01f, 0));

	data_list[1].MyTranslate(Eigen::Vector3f(-2, -6, 0));
	data_list[1].setSpeed(Eigen::Vector3f(0, 0.01f, 0));

	data_list[2].MyTranslate(Eigen::Vector3f(-6, -6, 0));
	data_list[2].setSpeed(Eigen::Vector3f(0.01f, 0, 0));

	data_list[3].MyTranslate(Eigen::Vector3f(6, 6, 0));
	data_list[3].setSpeed(Eigen::Vector3f(0, -0.01f, 0));

	data_list[4].MyTranslate(Eigen::Vector3f(-9, 5, 0));
	data_list[4].setSpeed(Eigen::Vector3f(0.01f, 0, 0));
	for (int i = 0; i < 5; i++) {
		data_list[i].set_visible(true);
	}
}

void MyViewer::level_ended() {
	stop_IK_solver_animation();
	timer.stop();
	level++;
	std::cout << "Level number: " << level << std::endl;
	std::cout << "The final score you achieved is: " << score << std::endl;
	Initialize_scene();
}

void MyViewer::Initialize_scene() {
	if (disp == NULL) {
		disp = new Display(1200, 1000, "Welcome");
	}
	MyRenderer renderer;
	load_configuration();
	init_simplify_data_structures_list();
	Init(*disp);
	renderer.init(this);
	renderer.my_init(this);
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);
	//delete disp;
}

