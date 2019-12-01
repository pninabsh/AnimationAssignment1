#pragma once

#include "igl/opengl/glfw/Display.h"
#include "igl/opengl/ViewerData.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "set"
#include <ctime>

typedef std::set<std::pair<double, int>> PriorityQueue;

struct SimplifyDataObject
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXi E;
	Eigen::MatrixXd F_NORMALS;
	std::vector<std::vector<int>> V_PLANES;
	std::vector< Eigen::Matrix4d> V_Q_MATRIX;
	Eigen::VectorXi EMAP;
	Eigen::MatrixXi EF;
	Eigen::MatrixXi EI;
	PriorityQueue Q;
	Eigen::MatrixXd C;
};

double calculate_edge_cost(Eigen::RowVectorXd new_v, Eigen::Matrix4d q_matrix_v1, Eigen::Matrix4d q_matrix_v2, int e);

Eigen::RowVector3d calculate_new_vertice_place(Eigen::MatrixXi E, Eigen::MatrixXd V,
	Eigen::Matrix4d& q_matrix_v1, Eigen::Matrix4d& q_matrix_v2, int e);

// init E,EMAP,EI,EF,C,Q, V_PLANES, V_Q_MATRIX
void get_SimplifyDataObject(SimplifyDataObject& simplifyDataObject);

// init V,F,E,EMAP,EI,EF,C,Q,Qit, V_PLANES, V_Q_MATRIX
SimplifyDataObject get_SimplifyDataObject(igl::opengl::ViewerData viewer_data);

bool collapse_edge(SimplifyDataObject& simplifyDataObject, std::vector<PriorityQueue::iterator >& Qit);

void update_q_matrixes(SimplifyDataObject& simplifyDataObject);
//Todo : create function to replace midpoint function

void update_priority_queue(SimplifyDataObject& simplifyDataObject);

void update_v_planes(SimplifyDataObject& simplifyDataObject);