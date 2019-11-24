#include "mesh_simplifier.h"

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

Eigen::Matrix4d do_sum_planes(SimplifyDataObject simplifyDataObject, Eigen::RowVector3d v_coordinates, std::vector<int> vertex_planes) {
	//go over all the planes and calculate: delta(v) = v^T * (sum of all Kp of each such plane) * v
	Eigen::Matrix4d sumOfAllPlanes = Eigen::Matrix4d::Zero();
	for (int plane_id : vertex_planes) {
		//Eigen::RowVectorXd plane_normal = viewer->data().F_normals.row(plane_id);
		Eigen::RowVectorXd plane_normal = simplifyDataObject.F_NORMALS.row(plane_id);
		sumOfAllPlanes += calculateKp(plane_normal, v_coordinates);
	}

	return sumOfAllPlanes;
}

Eigen::Matrix4d calculate_Qmatrix(SimplifyDataObject simplifyDataObject, int v_id) {
	std::vector<int> vertex_planes = simplifyDataObject.V_PLANES[v_id];
	Eigen::RowVector3d v_coordinates = simplifyDataObject.V.row(v_id);
	return do_sum_planes(simplifyDataObject, v_coordinates, vertex_planes);
}

Eigen::Matrix4d calculate_midpoint_Qmatrix(SimplifyDataObject simplifyDataObject, int e, int v1_id, int v2_id) {
	int F1 = simplifyDataObject.EF(e, 0);
	int F2 = simplifyDataObject.EF(e, 1);

	std::vector<int> vertex_planes = { F1,F2 };
	Eigen::RowVector3d midpoint_coordinates = (simplifyDataObject.V.row(v1_id) + simplifyDataObject.V.row(v2_id)) / 2;
	return 	do_sum_planes(simplifyDataObject, midpoint_coordinates, vertex_planes);
}

double calculate_vertex_cost(Eigen::Matrix<double, 4, 1> vectorMatrix, Eigen::Matrix4d q_matrix) {
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * q_matrix * vectorMatrix;
	return mulResult(0);
}

double calculate_edge_cost(SimplifyDataObject simplifyDataObject, int e) {
	int v1 = simplifyDataObject.E(e, 0);
	int v2 = simplifyDataObject.E(e, 1);

	Eigen::Matrix4d q_matrix_v1 = simplifyDataObject.V_Q_MATRIX[v1];

	Eigen::Matrix4d q_matrix_v2 = simplifyDataObject.V_Q_MATRIX[v2];

	Eigen::Matrix4d q_matrix_midpoint = calculate_midpoint_Qmatrix(simplifyDataObject, e, v1, v2);

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

Eigen::RowVector3d calculate_new_vertice_place() {
	Eigen::RowVectorXd p(1, 3);
	return p;
}

std::vector<SimplifyDataObject> get_simplify_data_structures_list(igl::opengl::glfw::Viewer* viewer){
	std::vector<igl::opengl::ViewerData> data_list = viewer->data_list;

	std::vector<SimplifyDataObject> simplifyDataObjectsList;

	for (int i = 0; i < data_list.size(); i++)
	{
		SimplifyDataObject simplifyDataObject;

		// init V,F
		simplifyDataObject.V = data_list[i].V;
		simplifyDataObject.F = data_list[i].F;

		// init E,EMAP,EF,EI
		igl::edge_flaps(simplifyDataObject.F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

		// init F_NORMALS
		simplifyDataObject.F_NORMALS.resize(simplifyDataObject.F.rows(), 3);
		for (int j = 0; j < simplifyDataObject.F_NORMALS.rows(); j++) {
			simplifyDataObject.F_NORMALS.row(j) = viewer->data(i).F_normals.row(j);
			std::cout << simplifyDataObject.F_NORMALS.row(j) << std::endl;
		}

		// init V_PLANES
		std::vector<std::vector<int>> V_PLANES;
		for (int j = 0; j < simplifyDataObject.V.rows(); j++) {
			V_PLANES.push_back(std::vector<int>());
		}

		for (int f = 0; f < simplifyDataObject.F.rows(); f++) {
			V_PLANES[simplifyDataObject.F(f, 0)].push_back(f);
			V_PLANES[simplifyDataObject.F(f, 1)].push_back(f);
			V_PLANES[simplifyDataObject.F(f, 2)].push_back(f);
		}

		simplifyDataObject.V_PLANES = V_PLANES;

		clock_t begin = clock();

		// init V_Q_MATRIX
		for (int v = 0; v < simplifyDataObject.V.rows(); v++) {
			simplifyDataObject.V_Q_MATRIX.push_back(calculate_Qmatrix(simplifyDataObject,v));
		}

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

		std::cout << elapsed_secs << std::endl;
		
		// init Q,C

		simplifyDataObject.C.resize(simplifyDataObject.E.rows(), simplifyDataObject.V.cols());

		simplifyDataObject.Q.clear();

		for (int e = 0; e < simplifyDataObject.E.rows(); e++)
		{

			//Todo: calculate new v' position and put it in C.row(e). implement it in 'calculate_new_vertice_place' method
			Eigen::RowVectorXd p(1, 3);
			simplifyDataObject.C.row(e) = p;

			double cost = calculate_edge_cost(simplifyDataObject, e);
			simplifyDataObject.Q.insert(std::pair<double, int>(0, e));
		}

		simplifyDataObjectsList.push_back(simplifyDataObject);

		clock_t end1 = clock();
		double elapsed_secs1 = double(end1 - end) / CLOCKS_PER_SEC;

		std::cout << elapsed_secs1 << std::endl;
	}

	return simplifyDataObjectsList;
};