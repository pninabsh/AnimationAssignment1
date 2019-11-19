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
static std::vector<SimplifyDataObject> get_simplify_data_structures_list(std::vector<igl::opengl::ViewerData>  data_list) {
	std::vector<SimplifyDataObject> simplifyDataObjectsList;

	for (int i = 0; i < data_list.size(); i++) {
		SimplifyDataObject simplifyDataObject;

		igl::edge_flaps(data_list[i].F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

		simplifyDataObject.C.resize(simplifyDataObject.E.rows(), data_list[i].V.cols());

		simplifyDataObject.Q.clear();

		for (int e = 0; e < simplifyDataObject.E.rows(); e++)
		{
			double cost = e;
			Eigen::RowVectorXd p(1, 3);
			simplifyDataObject.C.row(0) = p;

			simplifyDataObject.Q.insert(std::pair<double, int>(cost, e));
		}


		simplifyDataObjectsList.push_back(simplifyDataObject);
	}

	return simplifyDataObjectsList;
}

static void simplify(double number_of_edges) {
	std::cout << number_of_edges << std::endl;
}