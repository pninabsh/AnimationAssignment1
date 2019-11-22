#pragma once

#include "igl/opengl/ViewerData.h"
#include "igl/edge_flaps.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/collapse_edge.h"
#include "set"

typedef std::set<std::pair<double, int>> PriorityQueue;

struct SimplifyDataObject
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXi E;
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi EF;
	Eigen::MatrixXi EI;
	PriorityQueue Q;
	Eigen::MatrixXd C;
};

static void do_simplify(igl::opengl::glfw::Viewer *viewer)
{
	std::vector<SimplifyDataObject> simplifyDataObjectsList;
	SimplifyDataObject selectedSimplifyDataObject;

	const auto get_simplify_data_structures_list = [](std::vector<igl::opengl::ViewerData> data_list) -> std::vector<SimplifyDataObject> {
		std::vector<SimplifyDataObject> simplifyDataObjectsList;

		for (int i = 0; i < data_list.size(); i++)
		{
			SimplifyDataObject simplifyDataObject;

			simplifyDataObject.V = data_list[i].V;
			simplifyDataObject.F = data_list[i].F;

			igl::edge_flaps(simplifyDataObject.F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);
			/*if (i == 0)
			{
				std::cout << "Here is the matrix simplifyDataObject.EMAP:\n"
						  << simplifyDataObject.EMAP.row(0) << std::endl;
				std::cout << "Here is the matrix simplifyDataObject.EMAP:\n"
						  << simplifyDataObject.EMAP.row(760) << std::endl;
				std::cout << "Here is the matrix simplifyDataObject.EMAP:\n"
						  << simplifyDataObject.EMAP.row(1520) << std::endl;
				std::cout << "Here is the matrix simplifyDataObject.EF:\n" << simplifyDataObject.EF.row(6) << std::endl;
					std::cout << "Here is the matrix simplifyDataObject.EF:\n" << simplifyDataObject.EF.row(2) << std::endl;
					std::cout << "Here is the matrix simplifyDataObject.EF:\n" << simplifyDataObject.EF.row(0) << std::endl;
			}*/
			simplifyDataObject.C.resize(simplifyDataObject.E.rows(), simplifyDataObject.V.cols());

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

	const auto simplify = [&viewer, &selectedSimplifyDataObject](double number_of_edges) -> void {
		//std::cout << number_of_edges << std::endl;

		bool something_collapsed = false;
		int num_collapsed = 0;

		// TODO: write calculate_edges_cost as part of step 9
		// selectedSimplifyDataObject.C = calculate_edges_cost(....)

		for (int i = 0; i < number_of_edges; i++)
		{
			// TODO: write collapse_edge as part of step 10
			//if (!collapse_edge(..){
			//		break;
			//	}
			break;
			something_collapsed = true;
			num_collapsed++;
		}

		if (something_collapsed)
		{
			viewer->data().clear();
			viewer->data().set_mesh(selectedSimplifyDataObject.V, selectedSimplifyDataObject.F);
			viewer->data().set_face_based(true);
		}
	};

	simplifyDataObjectsList = get_simplify_data_structures_list(viewer->data_list);
	selectedSimplifyDataObject = simplifyDataObjectsList[viewer->selected_data_index];
	double rounded_up_five_percent_edges = std::ceil(0.05 * selectedSimplifyDataObject.E.rows());
	simplify(rounded_up_five_percent_edges);
}

Eigen::Matrix4d calculateKp(int f, igl::opengl::glfw::Viewer* viewer, Eigen::RowVector3d v) {
	//calculate a,b,c,d such that a*x + b*y + c*z + d = 0 and a^2 + b^2 + c^2 = 1
	const auto face_normal = viewer->data().F_normals.row(f).normalized();
	const auto a = face_normal(0);
	const auto b = face_normal(1);
	const auto c = face_normal(2);
	//calculaye d such that d = -a*x-b*y-c*z
	const auto d = -v(0) * a + -v(1) * b + -v(2) * c;
	Eigen::Matrix4d result;
	result << (a * a, a * b, a * c, a * d,
		a * b, b * b, b * c, b * d,
		a * c, b * c, c * c, c * d,
		a * d, b * d, c * d, d * d);
	return result;
}

bool is_face_partof_vertex(Eigen::RowVector3d v, int f, SimplifyDataObject simplifyDataObject) {
	int number_faces = simplifyDataObject.EMAP.size() / 3;
	bool isMemberOfFace = false;


	for (int i = 0; i < 3 || !isMemberOfFace; i++) {
		int index_edge_i = simplifyDataObject.EMAP(i * number_faces + f);

		int v0Index = simplifyDataObject.E(index_edge_i, 0);
		int v1Index = simplifyDataObject.E(index_edge_i, 1);

		Eigen::RowVector3d v0 = simplifyDataObject.V.row(v0Index);
		Eigen::RowVector3d v1 = simplifyDataObject.V.row(v1Index);

		const auto& isVerticeEqual = [](Eigen::RowVector3d a, Eigen::RowVector3d b) -> bool {
			return (a(0) == b(0) && a(1) == b(1) && a(2) == b(2));
		};
	
		if (isVerticeEqual(v0, v) || isVerticeEqual(v1, v)) {
			isMemberOfFace = true;
		}
	}

	return isMemberOfFace;

}

double calculate_vertex_cost(SimplifyDataObject simplifyDataObject, igl::opengl::glfw::Viewer* viewer, Eigen::RowVector3d v) {
	std::vector<int> vertex_planes;
	//find all planes that belong to this vertex v
	int number_faces = simplifyDataObject.EMAP.size() / 3;
	for (int f = 0; f < number_faces; f++) {
		if (is_face_partof_vertex(v, f, simplifyDataObject)) {
			vertex_planes.push_back(f);
		}
	}
	//go over all the planes and calculate: delta(v) = v^T * (sum of all Kp of each such plane) * v
	Eigen::Matrix4d sumOfAllPlanes = Eigen::Matrix4d::Zero();
	for (int i = 0; i < vertex_planes.size(); i++) {
		sumOfAllPlanes += calculateKp(vertex_planes[i], viewer, v);
	}
	Eigen::Matrix<double, 4, 1> vectorMatrix = { v(0), v(1), v(2), 1 };
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * sumOfAllPlanes * vectorMatrix;
	return mulResult(0);
}