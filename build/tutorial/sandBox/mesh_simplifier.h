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

		simplifyDataObject.Q.clear();

		simplifyDataObject.C.resize(simplifyDataObject.E.rows(), data_list[i].V.cols());

		for (int e = 0; e < simplifyDataObject.E.rows(); e++)
		{
			double cost = e;
			Eigen::RowVectorXd p(1, 3);
			simplifyDataObject.C.row(e) = p;
			simplifyDataObject.Q.insert(std::pair<double, int>(cost, e));
		}



		igl::edge_flaps(data_list[i].F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

		simplifyDataObjectsList.push_back(simplifyDataObject);
			
	}
}

static void simplify(int number_of_faces) {

}