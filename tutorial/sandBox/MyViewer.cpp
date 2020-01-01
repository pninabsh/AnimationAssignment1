#include "MyViewer.h"

#include <iostream>
#include <fstream>
using namespace std;

void MyViewer::load_configuration()
{
	cout << "reading mesh paths from configuration.txt file..." << endl;

	string mesh_path;
	ifstream configuration_file("configuration.txt");

	if (configuration_file.fail())
	{
		cout << "cannot open configuration file! loading from default mesh paths instead..." << endl;
		mesh_path = "C:/Dev/EngineIGLnew/tutorial/data/cube.obj";
	}
	else
	{
		while (configuration_file.good())
		{
			cout << "loading mesh from text line in configuration file..." << endl;
			getline(configuration_file, mesh_path);
		}
	}
	cout << "loading done!" << endl;
	configuration_file.close();

	load_mesh_from_file(mesh_path);
	load_mesh_from_file(mesh_path);

	this->data_list[0].MyScale(Eigen::RowVector3f(0.5, 0.5, 1));
	this->data_list[1].MyScale(Eigen::RowVector3f(0.5, 0.5, 1));

	this->data_list[0].setSpeed(Eigen::RowVector3f(0.001f, 0, 0));

	this->data_list[1].MyTranslate(Eigen::RowVector3f(0.75, 0, 0));
}