#pragma once

#include "igl/opengl/ViewerData.h"
#include "igl/edge_flaps.h"
#include "set"

typedef std::set<std::pair<double, int> > PriorityQueue;

struct SimplifyDataObject {
	Eigen::MatrixXi  E;
	Eigen::VectorXi  EMAP;
	Eigen::MatrixXi  EF;
	Eigen::MatrixXi  EI;
	PriorityQueue  Q;
	Eigen::MatrixXd  C;
};

static void do_simplify(igl::opengl::glfw::Viewer *viewer) {
	const auto get_simplify_data_structures_list = [](std::vector<igl::opengl::ViewerData>  data_list) -> std::vector<SimplifyDataObject> {
		std::vector<SimplifyDataObject> simplifyDataObjectsList;

		for (int i = 0; i < data_list.size(); i++) {
			SimplifyDataObject simplifyDataObject;

			igl::edge_flaps(data_list[i].F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

			simplifyDataObject.C.resize(simplifyDataObject.E.rows(), data_list[i].V.cols());

			simplifyDataObject.Q.clear();

			for (int e = 0; e < simplifyDataObject.E.rows(); e++)
			{
				Eigen::RowVectorXd p(1, 3);
				simplifyDataObject.C.row(e) = p;

				double cost = e;
				simplifyDataObject.Q.insert(std::pair<double, int>(cost, e));
			}


			simplifyDataObjectsList.push_back(simplifyDataObject);
		}
		
		return simplifyDataObjectsList;
	};

	const auto simplify = [](double number_of_edges) -> void {
		std::cout << number_of_edges << std::endl;
	};

	std::vector<SimplifyDataObject> simplifyDataObjectsList = get_simplify_data_structures_list(viewer->data_list);
	SimplifyDataObject simplifyDataObject = simplifyDataObjectsList[viewer->selected_data_index];
	double rounded_up_five_percent_edges = std::ceil(0.05 * simplifyDataObject.E.rows());
	simplify(rounded_up_five_percent_edges);
}

static std::vector<SimplifyDataObject> get_simplify_data_structures_list(std::vector<igl::opengl::ViewerData>  data_list) {
	std::vector<SimplifyDataObject> simplifyDataObjectsList;

	for (int i = 0; i < data_list.size(); i++) {
		SimplifyDataObject simplifyDataObject;

		igl::edge_flaps(data_list[i].F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

		simplifyDataObject.C.resize(simplifyDataObject.E.rows(), data_list[i].V.cols());

		simplifyDataObject.Q.clear();

		for (int e = 0; e < simplifyDataObject.E.rows(); e++)
		{
			Eigen::RowVectorXd p(0, 0);
			simplifyDataObject.C.row(e) = p;

			double cost = e;
			simplifyDataObject.Q.insert(std::pair<double, int>(cost, e));
		}


		simplifyDataObjectsList.push_back(simplifyDataObject);
	}

	return simplifyDataObjectsList;
}

static void simplify(double number_of_edges) {
	std::cout << number_of_edges << std::endl;
}