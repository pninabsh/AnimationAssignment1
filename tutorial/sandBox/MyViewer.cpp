#include "MyViewer.h"

#include <iostream>
#include <fstream>
using namespace std;

Eigen::RowVector3d black(0, 0, 0);
Eigen::RowVector3d grey(0.5, 0.5, 0.5);
Eigen::RowVector3d blue(0, 0, 1);
Eigen::RowVector3d green(0, 1, 0);
Eigen::RowVector3d teal(0, 1, 1);
Eigen::RowVector3d red(1, 0, 0);
Eigen::RowVector3d pink(1, 0, 1);
Eigen::RowVector3d yellow(1, 1, 0);
Eigen::RowVector3d white(1, 1, 1);

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

	this->data_list[1].MyTranslate(Eigen::RowVector3f(0.75, 0, 0));

	igl::AABB<Eigen::MatrixXd, 3> tree;
	igl::AABB<Eigen::MatrixXd, 3> *pointer;
	tree.init(this->data_list[0].V, this->data_list[0].F);
	std::cout << "woho" << std::endl;
	Eigen::AlignedBox<double, 3> right_face = tree.m_left->m_left->m_left->m_right->m_box;
	//Eigen::AlignedBox<double, 3> back_face = tree.m_right->m_left->m_right->m_box;
	//Eigen::AlignedBox<double, 3> bottom_face = tree.m_left->m_right->m_right->m_box;
	//Eigen::AlignedBox<double, 3> front_face = tree.m_left->m_left->m_right->m_box;
	//Eigen::AlignedBox<double, 3> top_face = tree.m_right->m_left->m_left->m_right->m_box;
	//Eigen::AlignedBox<double, 3> left_face = tree.m_left->m_left->m_left->m_right->m_box;
	//Eigen::RowVector3d v1 = right_face.corner(right_face.BottomLeftCeil);
	Eigen::RowVector3d v1 = right_face.corner(right_face.BottomLeftCeil);
	Eigen::RowVector3d v2 = right_face.corner(right_face.BottomLeftFloor);
	Eigen::RowVector3d v3 = right_face.corner(right_face.BottomRightCeil);
	Eigen::RowVector3d v4 = right_face.corner(right_face.BottomRightFloor);
	Eigen::RowVector3d v5 = right_face.corner(right_face.TopRightCeil);
	Eigen::RowVector3d v6 = right_face.corner(right_face.TopRightFloor);
	Eigen::RowVector3d v7 = right_face.corner(right_face.TopLeftCeil);
	Eigen::RowVector3d v8 = right_face.corner(right_face.TopLeftFloor);
	Eigen::RowVector3d v9 = right_face.center();

	this->data_list[0].add_points(v1, red);
	this->data_list[0].add_points(v2, green);
	this->data_list[0].add_points(v3, blue);
	this->data_list[0].add_points(v4, pink);
	this->data_list[0].add_points(v5, yellow);
	this->data_list[0].add_points(v6, white);
	this->data_list[0].add_points(v7, black);
	this->data_list[0].add_points(v8, teal);
	this->data_list[0].add_points(v9, grey);
}