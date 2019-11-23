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
	return helping_vector;
}

//calculate a,b,c,d such that a*x + b*y + c*z + d = 0 and a^2 + b^2 + c^2 = 1
Eigen::Matrix4d calculateKp(Eigen::RowVectorXd plane_normal, Eigen::RowVector3d v) {
	const auto a = plane_normal(0);
	const auto b = plane_normal(1);
	const auto c = plane_normal(2);
	//calculaye d such that d = -a*x-b*y-c*z
	const auto d = -v(0) * a + -v(1) * b + -v(2) * c;
	Eigen::Matrix4d result;
	result << a * a, a * b, a * c, a * d,
		a * b, b * b, b * c, b * d,
		a * c, b * c, c * c, c * d,
		a * d, b * d, c * d, d * d;
	return result;
}

bool is_face_partof_vertex(int v_id, int f_id, SimplifyDataObject simplifyDataObject) {
	return simplifyDataObject.F(f_id, 0) == v_id || simplifyDataObject.F(f_id, 1) == v_id || simplifyDataObject.F(f_id, 2) == v_id;
}

Eigen::Matrix4d do_sum_planes(igl::opengl::glfw::Viewer* viewer, Eigen::RowVector3d v_coordinates, std::vector<int> vertex_planes) {
	//go over all the planes and calculate: delta(v) = v^T * (sum of all Kp of each such plane) * v
	Eigen::Matrix4d sumOfAllPlanes = Eigen::Matrix4d::Zero();
	for (int plane_id : vertex_planes) {
		Eigen::RowVectorXd plane_normal = viewer->data().F_normals.row(plane_id);
		sumOfAllPlanes += calculateKp(plane_normal, v_coordinates);
	}

	return sumOfAllPlanes;
}

Eigen::Matrix4d calculate_Qmatrix(igl::opengl::glfw::Viewer* viewer, SimplifyDataObject simplifyDataObject, int v_id) {
	std::vector<int> vertex_planes;
	//find all planes that belong to this vertex v
	int number_faces = simplifyDataObject.F.rows();

	for (int f = 0; f < number_faces; f++) {
		if (is_face_partof_vertex(v_id, f, simplifyDataObject)) {
			vertex_planes.push_back(f);
		}
	}

	return do_sum_planes(viewer, simplifyDataObject.V.row(v_id), vertex_planes);
}

Eigen::Matrix4d calculate_midpoint_Qmatrix(igl::opengl::glfw::Viewer* viewer, SimplifyDataObject simplifyDataObject, int e, int v1_id, int v2_id) {
	int F1 = simplifyDataObject.EF(e, 0);
	int F2 = simplifyDataObject.EF(e, 1);

	std::vector<int> vertex_planes;
	vertex_planes.push_back(F1);
	vertex_planes.push_back(F2);

	return 	do_sum_planes(viewer, (simplifyDataObject.V.row(v1_id) + simplifyDataObject.V.row(v2_id)) / 2, vertex_planes);
}

double calculate_vertex_cost(Eigen::Matrix<double, 4, 1> vectorMatrix, Eigen::Matrix4d q_matrix) {
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * q_matrix * vectorMatrix;
	return mulResult(0);
}

double calculate_edge_cost(igl::opengl::glfw::Viewer* viewer, SimplifyDataObject simplifyDataObject, int e) {
	int v1 = simplifyDataObject.E(e, 0);
	int v2 = simplifyDataObject.E(e, 1);

	Eigen::Matrix4d q_matrix_v1 = calculate_Qmatrix(viewer, simplifyDataObject, v1);
	Eigen::Matrix4d q_matrix_v2 = calculate_Qmatrix(viewer, simplifyDataObject, v2);
	Eigen::Matrix4d q_matrix_midpoint = calculate_midpoint_Qmatrix(viewer, simplifyDataObject, e, v1, v2);

	std::vector<Eigen::Matrix4d> possible_qmatrices;
	possible_qmatrices.push_back(q_matrix_v1);
	possible_qmatrices.push_back(q_matrix_v2);
	possible_qmatrices.push_back(q_matrix_midpoint);

	Eigen::Matrix<double, 4, 1> optimal_vertex;
	double minimal_error = std::numeric_limits<double>::infinity();
	for (Eigen::Matrix4d possible_qmatrice : possible_qmatrices) {
		Eigen::Matrix<double, 4, 1> newVertex = calculate_new_contraction_vertex_position(possible_qmatrice);
		double error = calculate_vertex_cost(newVertex, possible_qmatrice);
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

			simplifyDataObject.C.resize(simplifyDataObject.E.rows(), simplifyDataObject.V.cols());

			simplifyDataObject.Q.clear();

			// for debug performance
			/*if (i != 0) {
				simplifyDataObjectsList.push_back(simplifyDataObject);
				continue;
			}*/

			for (int e = 0; e < simplifyDataObject.E.rows(); e++)
			{

				//Todo: calculate new v' position and put it in C.row(e)
				Eigen::RowVectorXd p(1, 3);
				simplifyDataObject.C.row(e) = p;

				double cost = calculate_edge_cost(viewer, simplifyDataObject, e);
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