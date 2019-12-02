#include "MyViewer.h"

#include <iostream>
#include <fstream>
using namespace std;


void MyViewer::load_configuration() {
	cout << "reading mesh paths from configuration.txt file..." << endl;

	ifstream configuration_file ("configuration.txt");


	if (configuration_file.fail()) {
		cout << "cannot open configuration file! loading from default mesh paths instead..." << endl;
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/cube.obj");
		load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/bunny.off");

	}
	else {
		string mesh_path;
		while (configuration_file.good()) {
			cout << "loading mesh from text line in configuration file..." << endl;
			getline(configuration_file, mesh_path);
			load_mesh_from_file(mesh_path);
		}
	}
	cout << "loading done!" << endl;
	configuration_file.close();
}

void MyViewer::init_simplify_data_structures_list() {
	for (igl::opengl::ViewerData viewer_data : data_list) {
		simplifyDataObjectsList->push_back(get_SimplifyDataObject(viewer_data));
	}
}

void MyViewer::simplify() {
	SimplifyDataObject selectedSimplifyDataObject;

	const auto do_simplify = [this,&selectedSimplifyDataObject](double number_of_edges) -> void {
		bool something_collapsed = false;
		int num_collapsed = 0;

		std::vector<PriorityQueue::iterator > Qit;
		Qit.resize(selectedSimplifyDataObject.E.rows());

		for (auto it = selectedSimplifyDataObject.Q.begin(); it != selectedSimplifyDataObject.Q.end(); ++it) {
			Qit[it->second] = it;
		};

		for (int i = 0; i < number_of_edges; i++)
		{
			if (!collapse_edge(selectedSimplifyDataObject, Qit)) {
				break;
			}
			something_collapsed = true;
			num_collapsed++;
			update_v_planes(selectedSimplifyDataObject);
			update_q_matrixes(selectedSimplifyDataObject);
			update_priority_queue(selectedSimplifyDataObject);
			for (auto it = selectedSimplifyDataObject.Q.begin(); it != selectedSimplifyDataObject.Q.end(); ++it) {
				Qit[it->second] = it;
			};
		}

		if (something_collapsed)
		{
			simplifyDataObjectsList->at(selected_data_index) = selectedSimplifyDataObject;
			data().clear();
			data().set_mesh(selectedSimplifyDataObject.V, selectedSimplifyDataObject.F);
			data().set_face_based(true);
			get_SimplifyDataObject(selectedSimplifyDataObject);
			// use the modified V and F and F_NORMALS to re-calculate E,EF,EI,Q,C,EMAP,V_Q_MATRIX,V_PLANES
		}
	};

	selectedSimplifyDataObject = simplifyDataObjectsList->at(selected_data_index);
	double rounded_up_five_percent_edges = std::ceil(0.05 * selectedSimplifyDataObject.Q.size());
	do_simplify(rounded_up_five_percent_edges);
}