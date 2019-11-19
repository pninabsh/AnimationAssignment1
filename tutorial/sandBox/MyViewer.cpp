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