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

Eigen::Matrix<double, 4, 1> calculate_new_contraction_vertex_position(Eigen::Matrix4d q_matrix) {
	Eigen::Matrix<double, 4, 1> helping_vector = { 0, 0, 0, 1 };
	/*return q_matrix.adjoint() * helping_vector;*/
	return helping_vector;
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
	result << a * a, a * b, a * c, a * d,
		a * b, b * b, b * c, b * d,
		a * c, b * c, c * c, c * d,
		a * d, b * d, c * d, d * d;
	return result;
}

bool is_face_partof_vertex(Eigen::RowVector3d v, int f, SimplifyDataObject simplifyDataObject) {
	int number_faces = simplifyDataObject.EMAP.size() / 3;
	bool isMemberOfFace = false;


	for (int i = 0; i < 3 && !isMemberOfFace; i++) {
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

Eigen::Matrix4d calculate_Qmatrix(igl::opengl::glfw::Viewer* viewer, SimplifyDataObject simplifyDataObject, Eigen::RowVector3d v) {
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
	return sumOfAllPlanes;
}

double calculate_vertex_cost(Eigen::Matrix<double, 4, 1> vectorMatrix, Eigen::Matrix4d q_matrix) {
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * q_matrix * vectorMatrix;
	return mulResult(0);
}

double calculate_edge_cost(SimplifyDataObject simplifyDataObject, igl::opengl::glfw::Viewer* viewer, int e) {
	int v0Index = simplifyDataObject.E(e, 0);
	int v1Index = simplifyDataObject.E(e, 1);
	std::vector<Eigen::Matrix4d> possible_qmatrices;
	Eigen::RowVector3d v1 = simplifyDataObject.V.row(v0Index);
	Eigen::RowVector3d v2 = simplifyDataObject.V.row(v1Index);
	Eigen::Matrix4d q_matrix_v1 = calculate_Qmatrix(viewer, simplifyDataObject, v1);
	Eigen::Matrix4d q_matrix_v2 = calculate_Qmatrix(viewer, simplifyDataObject, v2);
	possible_qmatrices.push_back(q_matrix_v1);
	possible_qmatrices.push_back(q_matrix_v2);
	possible_qmatrices.push_back(calculate_Qmatrix(viewer, simplifyDataObject, (v2 + v2) / 2));
	Eigen::Matrix<double, 4, 1> optimal_vertex;
	double minimal_error = std::numeric_limits<double>::max();
	for (int i = 0; i < possible_qmatrices.size(); i++) {
		Eigen::Matrix<double, 4, 1> newVertex = calculate_new_contraction_vertex_position(possible_qmatrices[i]);
		double error = calculate_vertex_cost(newVertex, possible_qmatrices[i]);
		if (error < minimal_error) {
			error = minimal_error;
			optimal_vertex = newVertex;
		}
	}
	Eigen::Matrix4d new_qmatrix = q_matrix_v1 + q_matrix_v2;
	return calculate_vertex_cost(optimal_vertex, new_qmatrix);

}

static void do_simplify(igl::opengl::glfw::Viewer* viewer)
{
	std::vector<SimplifyDataObject> simplifyDataObjectsList;
	SimplifyDataObject selectedSimplifyDataObject;

	const auto get_simplify_data_structures_list = [](igl::opengl::glfw::Viewer* viewer) -> std::vector<SimplifyDataObject> {
		std::vector<igl::opengl::ViewerData> data_list = viewer->data_list;
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

			for (int e = 0; e < 10; e++)
			{
				Eigen::RowVectorXd p(1, 3);
				simplifyDataObject.C.row(e) = p;

				double cost = calculate_edge_cost(simplifyDataObject, viewer, e);
				simplifyDataObject.Q.insert(std::pair<double, int>(0, e));
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

	simplifyDataObjectsList = get_simplify_data_structures_list(viewer);
	selectedSimplifyDataObject = simplifyDataObjectsList[viewer->selected_data_index];
	double rounded_up_five_percent_edges = std::ceil(0.05 * selectedSimplifyDataObject.E.rows());
	simplify(rounded_up_five_percent_edges);
}