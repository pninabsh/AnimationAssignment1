#include "MyViewer.h"

#include <iostream>
#include <fstream>
using namespace std;

void MyViewer::create_bounding_box() {
	igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(this->data_list[0].V, this->data_list[0].F);
	for(int i=0;i<this->data_list.size();i++) {
		tree.init(this->data_list[i].V, this->data_list[i].F);
		this->data_list[i].kd_tree = tree;
		Eigen::AlignedBox<double, 3> bounding_box = tree.m_box;
		draw_box(this->data_list[i], bounding_box, Eigen::RowVector3d(1, 0, 0));
	}
}

void MyViewer::load_configuration()
{
	cout << "reading mesh paths from configuration.txt file..." << endl;

	string mesh_path;
	ifstream configuration_file("configuration.txt");

	if (configuration_file.good())
	{
		cout << "loading mesh from text line in configuration file..." << endl;
		getline(configuration_file, mesh_path);
	}
	else {
		cout << "cannot open configuration file! loading from default mesh paths instead..." << endl;
		mesh_path = "C:/Dev/EngineIGLnew/tutorial/data/cube.obj";
	}

	cout << "loading done!" << endl;
	configuration_file.close();

	load_mesh_from_file(mesh_path);
	load_mesh_from_file(mesh_path);

	this->data_list[0].MyScale(Eigen::RowVector3f(0.5, 0.5, 1));
	this->data_list[1].MyScale(Eigen::RowVector3f(0.5, 0.5, 1));

	this->data_list[0].setSpeed(Eigen::RowVector3f(0.001f, 0, 0));

	this->data_list[1].MyTranslate(Eigen::RowVector3f(1.75, 0, 0));
	
	create_bounding_box();
}