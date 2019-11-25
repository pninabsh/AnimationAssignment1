#include "mesh_simplifier.h"

std::pair<double, int> get_lowest_cost_and_edge_pair(PriorityQueue Q) {
	std::pair<double, int> selectedCostAndEdge = std::pair<double, int>(std::numeric_limits<double>::infinity(), -1);
	for (std::pair<double, int> costAndEdge : Q) {
		if (selectedCostAndEdge.first > costAndEdge.first) {
			selectedCostAndEdge = costAndEdge;
		}
	}

	return selectedCostAndEdge;
}

void erase_face(Eigen::MatrixXi& F, Eigen::MatrixXd& F_NORMALS, int face_id) {
	Eigen::MatrixXi new_F;
	Eigen::MatrixXd new_F_NORMALS;

	int j = 0;
	for (int i = 0; i < F.rows() && j < F.rows(); i++, j++) {
		if (j == face_id) {
			j++;
		}

		if (j == F.rows()) {
			break;
		}

		new_F.row(i) = F.row(j);
		new_F_NORMALS.row(i) = F_NORMALS.row(j);
	}

	F = new_F;
	F_NORMALS = new_F_NORMALS;
}

void erase_vertice(Eigen::MatrixXd& V, int vertice_id) {
	Eigen::MatrixXd new_V;

	int j = 0;
	for (int i = 0; i < V.rows() && j < V.rows(); i++, j++) {
		if (j == vertice_id) {
			j++;
		}

		if (j == V.rows()) {
			break;
		}

		new_V.row(i) = V.row(j);
	}

	V = new_V;
}


void replace_vertice_in_all_faces(Eigen::MatrixXi& F, int old_vertice_id, int new_vertice_id) {
	for (int i = 0; i < F.rows(); i++) {
		for (int j = 0; j < F.cols(); j++) {
			if (F(i, j) == old_vertice_id) {
				F(i, j) = new_vertice_id;
			}
		}
	}
}

// 1. given: edge with index 'e' and cost value 'cost' with the lowest cost value in Q
// 2. given: EF(e,0)=F1 and EF(e,1)=F2
// 3. delete: F.row(F1) and F.row(F2)
// 4. given: E(e,0)=Vs and E(e,1)=Vd
// 5. delete: V.row(Vd)
// 6. do: V.row(Vs)=C.row(e)
// 7. for each cell in F: if F(i,j) == Vd then F(i,j)= Vs
// 8. print: "edge ${e}, cost = ${cost}, new v position (${C.row(e,0)},${C.row(e,1)},${C.row(e,2)})"

bool collapse_edge(SimplifyDataObject& simplifyDataObject) {
	// 1. given: edge with index 'e' and cost value 'cost' with the lowest cost value in Q
	std::pair<double, int> selectedCostAndEdge = get_lowest_cost_and_edge_pair(simplifyDataObject.Q);

	if (selectedCostAndEdge.second == -1) {
		return false;
	}

	simplifyDataObject.Q.erase(selectedCostAndEdge);

	double cost = selectedCostAndEdge.first;
	int e = selectedCostAndEdge.second;

	// 2. given: EF(e,0)=F1 and EF(e,1)=F2

	int F1 = simplifyDataObject.EF(e, 0);
	int F2 = simplifyDataObject.EF(e, 1);

	// 3. delete: F.row(F1) and F.row(F2)

	erase_face(simplifyDataObject.F, simplifyDataObject.F_NORMALS,F1);
	erase_face(simplifyDataObject.F, simplifyDataObject.F_NORMALS,F2);

	// 4. given: E(e,0)=Vs and E(e,1)=Vd

	int Vs = simplifyDataObject.E(e, 0);
	int Vd = simplifyDataObject.E(e, 1);

	// 5. delete: V.row(Vd)

	erase_vertice(simplifyDataObject.V, Vd);

	// 6. do: V.row(Vs)=C.row(e)

	simplifyDataObject.V.row(Vs) = simplifyDataObject.C.row(e);

	// 7. for each cell in F: if F(i,j) == Vd then F(i,j)= Vs

	replace_vertice_in_all_faces(simplifyDataObject.F, Vd, Vs);

	// 8. print: "edge ${e}, cost = ${cost}, new v position (${C(e,0)},${C(e,1)},${C(e,2)})"

	std::cout << "edge " << e << ", cost = " << cost << ", new v position (" << simplifyDataObject.C(e, 0) << ","
		<< simplifyDataObject.C(e, 1) << "," << simplifyDataObject.C(e, 2) << ")" << std::endl;

	return true;

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

 // init E,EMAP,EI,EF,C,Q, V_PLANES, V_Q_MATRIX
 void get_SimplifyDataObject(SimplifyDataObject& simplifyDataObject){
	 // init E,EMAP,EF,EI
	 igl::edge_flaps(simplifyDataObject.F, simplifyDataObject.E, simplifyDataObject.EMAP, simplifyDataObject.EF, simplifyDataObject.EI);

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
		 simplifyDataObject.V_Q_MATRIX.push_back(calculate_Qmatrix(simplifyDataObject, v));
	 }


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
 }

 // init V,F,E,EMAP,EI,EF,C,Q, V_PLANES, V_Q_MATRIX
SimplifyDataObject get_SimplifyDataObject(igl::opengl::ViewerData viewer_data){
		SimplifyDataObject simplifyDataObject;

		// init V,F
		simplifyDataObject.V = viewer_data.V;
		simplifyDataObject.F = viewer_data.F;

		// init F_NORMALS
		simplifyDataObject.F_NORMALS.resize(simplifyDataObject.F.rows(), 3);
		for (int j = 0; j < simplifyDataObject.F_NORMALS.rows(); j++) {
			simplifyDataObject.F_NORMALS.row(j) = viewer_data.F_normals.row(j);
			std::cout << simplifyDataObject.F_NORMALS.row(j) << std::endl;
		}


		// init E,EMAP,EI,EF,C,Q, V_PLANES, V_Q_MATRIX
		get_SimplifyDataObject(simplifyDataObject);

		return  simplifyDataObject;
};