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

		for (int i = 0; i < number_of_edges; i++)
		{
			if (!collapse_edge(selectedSimplifyDataObject)) {
				// use the modified V and F and F_NORMALS to re-calculate E,EF,EI,Q,C,EMAP,V_Q_MATRIX,V_PLANES
				break;
			}
			// use the modified V and F and F_NORMALS to re-calculate E,EF,EI,Q,C,EMAP,V_Q_MATRIX,V_PLANES
			get_SimplifyDataObject(selectedSimplifyDataObject);
			something_collapsed = true;
			num_collapsed++;
		}

		if (something_collapsed)
		{
			data().clear();
			data().set_mesh(selectedSimplifyDataObject.V, selectedSimplifyDataObject.F);
			data().set_face_based(true);
		}
	};

	selectedSimplifyDataObject = simplifyDataObjectsList->at(selected_data_index);
	double rounded_up_five_percent_edges = std::ceil(0.05 * selectedSimplifyDataObject.E.rows());
	do_simplify(rounded_up_five_percent_edges);
}