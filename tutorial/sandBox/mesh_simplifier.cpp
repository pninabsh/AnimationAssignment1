#include "mesh_simplifier.h"
#include "igl/collapse_edge.h"

std::vector<int> get_v_faces(Eigen::MatrixXi F, int v) {
	std::vector<int> v_faces = std::vector<int>();
	for (int f = 0; f < F.rows(); f++) {
		if (F(f, 0) == IGL_COLLAPSE_EDGE_NULL || F(f, 1) == IGL_COLLAPSE_EDGE_NULL || F(f, 2) == IGL_COLLAPSE_EDGE_NULL) {
			continue;
		}

		if (F(f, 0) == v || F(f, 1) == v || F(f, 2) == v) {
			v_faces.push_back(f);
		}
	}

	return v_faces;
}

//calculate a,b,c,d such that a*x + b*y + c*z + d = 0 and a^2 + b^2 + c^2 = 1
Eigen::Matrix4d calculateKp(Eigen::RowVectorXd plane_normal, Eigen::RowVector3d v)
{
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

Eigen::Matrix4d do_sum_planes(Eigen::MatrixXd F_NORMALS, Eigen::RowVector3d v_coordinates, std::vector<int> vertex_planes)
{
	//go over all the planes and calculate: delta(v) = v^T * (sum of all Kp of each such plane) * v
	Eigen::Matrix4d sumOfAllPlanes = Eigen::Matrix4d::Zero();
	for (int plane_id : vertex_planes)
	{
		//Eigen::RowVectorXd plane_normal = viewer->data().F_normals.row(plane_id);
		Eigen::RowVectorXd plane_normal = F_NORMALS.row(plane_id);
		sumOfAllPlanes += calculateKp(plane_normal, v_coordinates);
	}

	return sumOfAllPlanes;
}

Eigen::Matrix4d calculate_Qmatrix(Eigen::MatrixXd V, std::vector<int> vertice_planes, Eigen::MatrixXd F_NORMALS, int v_id)
{
	//std::vector<int> vertex_planes = V_PLANES[v_id];
	Eigen::RowVector3d v_coordinates = V.row(v_id);
	return do_sum_planes(F_NORMALS, v_coordinates, vertice_planes);
}

double calculate_vertex_cost(Eigen::Matrix<double, 4, 1> vectorMatrix, Eigen::Matrix4d q_matrix)
{
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * q_matrix * vectorMatrix;
	return mulResult(0);
}

//function that calcutes v' for each edge e
Eigen::RowVector3d calculate_new_vertice_place(Eigen::MatrixXi E, Eigen::MatrixXd V, Eigen::Matrix4d q_matrix_v1, Eigen::Matrix4d q_matrix_v2, int e)
{
	Eigen::Matrix4d q_matrix_new_vertex = q_matrix_v1 + q_matrix_v2;
	q_matrix_new_vertex(3, 0) = 0; q_matrix_new_vertex(3, 1) = 0; q_matrix_new_vertex(3, 2) = 0; q_matrix_new_vertex(3, 3) = 1;
	//check if this matrix is inversible, if so we need to multiply q' with helping vector, otherwise return midpoint
	if (q_matrix_new_vertex.determinant() != 0) {
		Eigen::RowVectorXd p(3);
		Eigen::Matrix<double, 4, 1> helping_vector = { 0, 0, 0, 1 };

		Eigen::Matrix<double, 4, 1> multiply_result_matrix = q_matrix_new_vertex.inverse() * helping_vector;

		p << multiply_result_matrix(0, 0), multiply_result_matrix(1, 0), multiply_result_matrix(2, 0);
		return p;
	}
	//otherwise return midpoint coordinates
	Eigen::RowVector3d midpoint_coordinates = (V.row(E(e, 0)) + V.row(E(e, 1))) / 2;
	return midpoint_coordinates;
}

double calculate_edge_cost(Eigen::RowVectorXd new_v, Eigen::Matrix4d q_matrix_v1, Eigen::Matrix4d q_matrix_v2, int e)
{
	Eigen::Matrix4d q_matrix = q_matrix_v1 + q_matrix_v2;
	Eigen::Vector4d vertex(new_v(0), new_v(1), new_v(2), 1);
	return calculate_vertex_cost(vertex, q_matrix);
}

// init E,EMAP,EI,EF,C,Q,Qit, V_PLANES, V_Q_MATRIX
void get_SimplifyDataObject(SimplifyDataObject &simplifyDataObject)
{
	// init E,EMAP,EF,EI
	igl::edge_flaps(simplifyDataObject.F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

	// init V_PLANES
	std::vector<std::vector<int>> V_PLANES;
	for (int j = 0; j < simplifyDataObject.V.rows(); j++)
	{
		V_PLANES.push_back(std::vector<int>());
	}

	for (int f = 0; f < simplifyDataObject.F.rows(); f++)
	{
		V_PLANES[simplifyDataObject.F(f, 0)].push_back(f);
		V_PLANES[simplifyDataObject.F(f, 1)].push_back(f);
		V_PLANES[simplifyDataObject.F(f, 2)].push_back(f);
	}

	simplifyDataObject.V_PLANES = V_PLANES;

	// init V_Q_MATRIX
	for (int v = 0; v < simplifyDataObject.V.rows(); v++)
	{
		simplifyDataObject.V_Q_MATRIX.push_back(calculate_Qmatrix(simplifyDataObject.V, simplifyDataObject.V_PLANES[v],simplifyDataObject.F_NORMALS, v));
	}

	// init Q,Qit,C

	simplifyDataObject.C.resize(simplifyDataObject.E.rows(), simplifyDataObject.V.cols());

	simplifyDataObject.Q.clear();

	for (int e = 0; e < simplifyDataObject.E.rows(); e++)
	{
		Eigen::Matrix4d q_matrix_v1 = simplifyDataObject.V_Q_MATRIX[simplifyDataObject.E(e, 0)];
		Eigen::Matrix4d q_matrix_v2 = simplifyDataObject.V_Q_MATRIX[simplifyDataObject.E(e, 1)];
		Eigen::RowVectorXd p = calculate_new_vertice_place(simplifyDataObject.E, simplifyDataObject.V, q_matrix_v1, q_matrix_v2, e);
		simplifyDataObject.C.row(e) = p;

		int v1 = simplifyDataObject.E(e, 0);
		int v2 = simplifyDataObject.E(e, 1);
		double cost = calculate_edge_cost(p, simplifyDataObject.V_Q_MATRIX[v1], simplifyDataObject.V_Q_MATRIX[v2], e);
		simplifyDataObject.Q.insert(std::pair<double, int>(cost, e));
	}
}

// init V,F,E,EMAP,EI,EF,C,Q, V_PLANES, V_Q_MATRIX
SimplifyDataObject get_SimplifyDataObject(igl::opengl::ViewerData viewer_data)
{
	SimplifyDataObject simplifyDataObject;

	// init V,F
	simplifyDataObject.V = viewer_data.V;
	simplifyDataObject.F = viewer_data.F;

	// init F_NORMALS
	simplifyDataObject.F_NORMALS.resize(simplifyDataObject.F.rows(), 3);
	for (int j = 0; j < simplifyDataObject.F_NORMALS.rows(); j++)
	{
		simplifyDataObject.F_NORMALS.row(j) = viewer_data.F_normals.row(j);
	}

	// init E,EMAP,EI,EF,C,Q,Qit, V_PLANES, V_Q_MATRIX
	get_SimplifyDataObject(simplifyDataObject);

	return simplifyDataObject;
};

bool collapse_edge(SimplifyDataObject& simplifyDataObject, std::vector<PriorityQueue::iterator > &Qit)
{
	const auto& cost_and_placement = [&](const int e,
		const Eigen::MatrixXd& V,
		const Eigen::MatrixXi& F,
		const Eigen::MatrixXi& E,
		const Eigen::VectorXi& EMAP,
		const Eigen::MatrixXi& EF,
		const Eigen::MatrixXi& EI,
		double& cost,
		Eigen::RowVectorXd& p) -> void {
			int v1 = E(e, 0);
			int v2 = E(e, 1);
			std::vector<int> v1_faces = get_v_faces(F, v1);
			std::vector<int> v2_faces = get_v_faces(F, v2);

			Eigen::Matrix4d q_matrix_v1 = calculate_Qmatrix(V, v1_faces, simplifyDataObject.F_NORMALS,v1);
			Eigen::Matrix4d q_matrix_v2 = calculate_Qmatrix(V, v2_faces, simplifyDataObject.F_NORMALS, v2);

			p = calculate_new_vertice_place(E,V, q_matrix_v1, q_matrix_v2, e);
			cost = calculate_edge_cost(p, q_matrix_v1, q_matrix_v2, e);
	};

	double cost = simplifyDataObject.Q.begin()->first;
	int e = simplifyDataObject.Q.begin()->second;
	Eigen::RowVectorXd new_v_location = simplifyDataObject.C.row(e);

	igl::collapse_edge(
		cost_and_placement, simplifyDataObject.V, simplifyDataObject.F,
		simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF,
		simplifyDataObject.EI, simplifyDataObject.Q, Qit, simplifyDataObject.C);

	std::cout << "edge " << e << ", cost = " << cost << ", new v position (" << new_v_location(0) << ","
		<< new_v_location(1) << "," << new_v_location(2) << ")" << std::endl;
	return true;
};