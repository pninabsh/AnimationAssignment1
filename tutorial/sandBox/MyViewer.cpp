#include "MyViewer.h"
#include <iostream>
#include <fstream>
using namespace std;

void MyViewer::load_configuration()
{
	cout << "reading mesh paths from configuration.txt file..." << endl;

	ifstream configuration_file("configuration.txt");

	if (configuration_file.fail())
	{
		cout << "cannot open configuration file! loading from default mesh paths instead..." << endl;
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/cube.obj");
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/bunny.off");
	}
	else
	{
		string mesh_path;
		while (configuration_file.good())
		{
			cout << "loading mesh from text line in configuration file..." << endl;
			getline(configuration_file, mesh_path);
			load_mesh_from_file(mesh_path);
		}
	}
	cout << "loading done!" << endl;
	configuration_file.close();
}

void fail_load_configuration_IK(string &sphere_path, string &yCylinder_path)
{
	cout << "cannot open or read all needed line from configuration file! loading from default mesh paths instead..." << endl;
	sphere_path = "C:/Dev/EngineIGLnew/tutorial/data/sphere.obj";
	yCylinder_path = "C:/Dev/EngineIGLnew/tutorial/data/ycylinder.obj";
}

bool attempt_to_load_IK_mesh(string &mesh_path, ifstream &configuration_file)
{
	if (configuration_file.good())
	{
		getline(configuration_file, mesh_path);
		return true;
	}
	return false;
}


void MyViewer::load_configuration_IK()
{
	string sphere_path;
	string yCylinder_path;
	cout << "reading mesh paths from configuration.txt file..." << endl;

	ifstream configuration_file("configuration.txt");

	if (configuration_file.fail())
	{
		fail_load_configuration_IK(sphere_path, yCylinder_path);
	}
	else
	{
		cout << "loading sphere and yCylinder mesh from text line in configuration file..." << endl;
		if (!attempt_to_load_IK_mesh(sphere_path, configuration_file) || !attempt_to_load_IK_mesh(yCylinder_path, configuration_file))
		{
			fail_load_configuration_IK(sphere_path, yCylinder_path);
		}
	}

	load_mesh_from_file(sphere_path);

	for (int i = 0; i < 10; i++) {
		load_mesh_from_file(yCylinder_path);
	}
	
	for (int i = 2; i <= 10; i++) {
		data_list[i].SetParent(&(data_list[i - 1]));
	}

	data_list[0].MyTranslate(Eigen::Vector3f(1, -1, -2));
	data_list[0].MyScale(Eigen::Vector3f(resize_value, resize_value, resize_value));
	data_list[0].setSpeed(Eigen::Vector3f (0, 0.01f, 0));
	data_list[1].MyScale(Eigen::Vector3f(resize_value, resize_value, resize_value));
	data_list[1].SetCenterOfRotation(Eigen::Vector3f(0, -1, 0));
	data_list[1].MyTranslate(Eigen::Vector3f(0, -1, -2));
	for (int i = 2; i <= 10; i++) {
		data_list[i].SetCenterOfRotation(Eigen::Vector3f(0, 0.8, 0));
	}
	for (int i = 2; i <= 10; i++) {
		data_list[i].MyTranslate(Eigen::Vector3f(0, 0.8, 0));
	}
	for (int i = 1; i < data_list.size(); i++) {
		links_numbers->push_back(i);

		if (i != 10) {
			parent_links_indices->push_back(links_numbers->at(links_numbers->size() - 1));
		}
	}

	cout << "loading done!" << endl;
	configuration_file.close();
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
		bool something_collapsed = false;

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
			something_collapsed = true;
			update_v_planes(selectedSimplifyDataObject);
			update_q_matrixes(selectedSimplifyDataObject);
			update_priority_queue(selectedSimplifyDataObject);
		}

		simplifyDataObjectsList->at(selected_data_index) = selectedSimplifyDataObject;

		if (something_collapsed)
		{
			data().clear();
			data().set_mesh(selectedSimplifyDataObject.V, selectedSimplifyDataObject.F);
			data().set_face_based(true);
			// use the modified V and F and F_NORMALS to re-calculate E,EF,EI,Q,C,EMAP,V_Q_MATRIX,V_PLANES
			get_SimplifyDataObject(selectedSimplifyDataObject);
		}
	};

	selectedSimplifyDataObject = simplifyDataObjectsList->at(selected_data_index);
	double rounded_up_five_percent_edges = std::ceil(0.05 * selectedSimplifyDataObject.Q.size());
	do_simplify(rounded_up_five_percent_edges);
}