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

void draw_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box, std::vector<Eigen::AlignedBox3d::CornerType> vertices) {
	Eigen::RowVector3d v1 = bounding_box.corner(vertices.at(0));
	Eigen::RowVector3d v2 = bounding_box.corner(vertices.at(1));
	Eigen::RowVector3d v3 = bounding_box.corner(vertices.at(2));
	Eigen::RowVector3d v4 = bounding_box.corner(vertices.at(3));
	mesh.add_edges(v1, v2, red);
	mesh.add_edges(v1, v3, red);
	mesh.add_edges(v2, v4, red);
	mesh.add_edges(v3, v4, red);
}

void draw_back_face(igl::opengl::ViewerData &mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopRightFloor,
		bounding_box.BottomLeftFloor, bounding_box.BottomRightFloor  });
}

void draw_front_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftCeil, bounding_box.TopRightCeil,
		bounding_box.BottomLeftCeil, bounding_box.BottomRightCeil });
}

void draw_top_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopRightFloor,
		bounding_box.TopLeftCeil, bounding_box.TopRightCeil  });
}

void draw_bottom_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.BottomLeftFloor, bounding_box.BottomRightFloor,
		bounding_box.BottomLeftCeil, bounding_box.BottomRightCeil  });
}

void draw_left_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopLeftFloor, bounding_box.TopLeftCeil,
		bounding_box.BottomLeftFloor, bounding_box.BottomLeftCeil  });
}

void draw_right_face(igl::opengl::ViewerData& mesh, Eigen::AlignedBox<double, 3> bounding_box) {
	draw_face(mesh, bounding_box, std::vector<Eigen::AlignedBox3d::CornerType>{bounding_box.TopRightFloor, bounding_box.TopRightCeil,
		bounding_box.BottomRightFloor, bounding_box.BottomRightCeil  });
}

void MyViewer::create_bounding_box() {
	igl::AABB<Eigen::MatrixXd, 3> tree;
	for(int i=0;i<this->data_list.size();i++) {
		tree.init(this->data_list[i].V, this->data_list[i].F);
		Eigen::AlignedBox<double, 3> bounding_box = tree.m_box;
		draw_back_face(this->data_list[i], bounding_box);
		draw_front_face(this->data_list[i], bounding_box);
		draw_top_face(this->data_list[i], bounding_box);
		draw_bottom_face(this->data_list[i], bounding_box);
		draw_left_face(this->data_list[i], bounding_box);
		draw_right_face(this->data_list[i], bounding_box);
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

	this->data_list[1].MyTranslate(Eigen::RowVector3f(0.75, 0, 0));
	
	create_bounding_box();

	/*igl::AABB<Eigen::MatrixXd, 3> tree;
	tree.init(this->data_list[0].V, this->data_list[0].F);
	Eigen::AlignedBox<double, 3> bounding_box = tree.m_box;
	Eigen::RowVector3d v1 = bounding_box.corner(bounding_box.TopLeftFloor);
	Eigen::RowVector3d v2 = bounding_box.corner(bounding_box.TopRightFloor);
	this->data_list[0].add_points(v1, red);*/
}