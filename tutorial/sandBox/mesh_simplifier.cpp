#include "mesh_simplifier.h"

bool collapse_edge(SimplifyDataObject &simplifyDataObject)
{

	
	// send to collapse_edge() with the error function instead of the midpont function;

	// re-calculate Qmatrix,Q,C

	// print message
	/*std::cout << "edge " << e << ", cost = " << cost << ", new v position (" << simplifyDataObject.C(e, 0) << ","
			  << simplifyDataObject.C(e, 1) << "," << simplifyDataObject.C(e, 2) << ")" << std::endl;
			  */
	return true;
};

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

Eigen::Matrix4d do_sum_planes(SimplifyDataObject simplifyDataObject, Eigen::RowVector3d v_coordinates, std::vector<int> vertex_planes)
{
	//go over all the planes and calculate: delta(v) = v^T * (sum of all Kp of each such plane) * v
	Eigen::Matrix4d sumOfAllPlanes = Eigen::Matrix4d::Zero();
	for (int plane_id : vertex_planes)
	{
		//Eigen::RowVectorXd plane_normal = viewer->data().F_normals.row(plane_id);
		Eigen::RowVectorXd plane_normal = simplifyDataObject.F_NORMALS.row(plane_id);
		sumOfAllPlanes += calculateKp(plane_normal, v_coordinates);
	}

	return sumOfAllPlanes;
}

Eigen::Matrix4d calculate_Qmatrix(SimplifyDataObject simplifyDataObject, int v_id)
{
	std::vector<int> vertex_planes = simplifyDataObject.V_PLANES[v_id];
	Eigen::RowVector3d v_coordinates = simplifyDataObject.V.row(v_id);
	return do_sum_planes(simplifyDataObject, v_coordinates, vertex_planes);
}

double calculate_vertex_cost(Eigen::Matrix<double, 4, 1> vectorMatrix, Eigen::Matrix4d q_matrix)
{
	Eigen::Matrix<double, 1, 1> mulResult = vectorMatrix.adjoint() * q_matrix * vectorMatrix;
	return mulResult(0);
}

double calculate_edge_cost(SimplifyDataObject simplifyDataObject, int e)
{
	Eigen::RowVectorXd p = simplifyDataObject.C.row(e);
	int v1 = simplifyDataObject.E(e, 0);
	int v2 = simplifyDataObject.E(e, 1);
	Eigen::Matrix4d q_matrix_v1 = simplifyDataObject.V_Q_MATRIX[v1];
	Eigen::Matrix4d q_matrix_v2 = simplifyDataObject.V_Q_MATRIX[v2];
	Eigen::Matrix4d q_matrix = q_matrix_v1 + q_matrix_v2;
	Eigen::Vector4d vertex(p(0), p(1), p(2), 1);
	return calculate_vertex_cost(vertex, q_matrix);
}

//function that calcutes v' for each edge e
Eigen::RowVector3d calculate_new_vertice_place(SimplifyDataObject& simplifyDataObject, int e)
{
	Eigen::RowVectorXd p(3);
	Eigen::Matrix<double, 4, 1> helping_vector = { 0, 0, 0, 1 };
	int v1 = simplifyDataObject.E(e, 0);
	int v2 = simplifyDataObject.E(e, 1);
	Eigen::RowVector3d midpoint_coordinates = (simplifyDataObject.V.row(v1) + simplifyDataObject.V.row(v2)) / 2;
	Eigen::Matrix4d q_matrix_v1 = simplifyDataObject.V_Q_MATRIX[v1];
	Eigen::Matrix4d q_matrix_v2 = simplifyDataObject.V_Q_MATRIX[v2];
	Eigen::Matrix4d q_matrix_new_vertex = q_matrix_v1 + q_matrix_v2;
	//check if this matrix is inversible, if so we need to multiply q' with helping vector, otherwise return midpoint
	if (q_matrix_new_vertex.determinant() != 0) {
		Eigen::Matrix<double, 4, 1> multiply_result_matrix = q_matrix_new_vertex.inverse() * helping_vector;
		double a = multiply_result_matrix(0, 0);
		double b = multiply_result_matrix(1, 0);
		double c = multiply_result_matrix(2, 0);
		p << a, b, c;
		return p;
	}
	//otherwise return midpoint coordinates
	return midpoint_coordinates;
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
		simplifyDataObject.V_Q_MATRIX.push_back(calculate_Qmatrix(simplifyDataObject, v));
	}

	// init Q,Qit,C

	simplifyDataObject.C.resize(simplifyDataObject.E.rows(), simplifyDataObject.V.cols());

	simplifyDataObject.Q.clear();

	simplifyDataObject.Qit.resize(simplifyDataObject.E.rows());

	for (int e = 0; e < simplifyDataObject.E.rows(); e++)
	{
		//Todo: calculate new v' position and put it in C.row(e). implement it in 'calculate_new_vertice_place' method
		Eigen::RowVectorXd p = calculate_new_vertice_place(simplifyDataObject, e);
		simplifyDataObject.C.row(e) = p;
		std::cout << e << "- start" << std::endl;
		double cost = calculate_edge_cost(simplifyDataObject, e);
		simplifyDataObject.Qit[e] = simplifyDataObject.Q.insert(std::pair<double, int>(cost, e)).first;
		std::cout << e << "- end" << std::endl;
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