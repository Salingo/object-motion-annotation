#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

inline Eigen::MatrixXd transform(std::string type, Eigen::Vector3f axispos, Eigen::Vector3f axisdir, float trans, float angle, Eigen::MatrixXd input_V)
{
	Eigen::MatrixXd V_trans(input_V.rows(), 3);
	Eigen::Affine3f transform;
	if (type == "T")
		transform = Eigen::Translation3f(axisdir * trans) * Eigen::AngleAxisf(0, axisdir);
	else if (type == "R")
		transform = Eigen::Translation3f(axispos) * Eigen::AngleAxisf(M_PI/180 * angle, axisdir) * Eigen::Translation3f(-axispos);
	else if (type == "TR")
		transform = Eigen::Translation3f(axisdir * trans) * Eigen::Translation3f(axispos) * Eigen::AngleAxisf(M_PI/180 * angle, axisdir) * Eigen::Translation3f(-axispos);
	else
		printf("motion type error");

	for (int i = 0; i < input_V.rows(); i++)
	{
		V_trans.row(i).x() = transform(0,0) * input_V.row(i).x() + transform(0,1) * input_V.row(i).y() + transform(0,2) * input_V.row(i).z() + transform(0,3);
		V_trans.row(i).y() = transform(1,0) * input_V.row(i).x() + transform(1,1) * input_V.row(i).y() + transform(1,2) * input_V.row(i).z() + transform(1,3);
		V_trans.row(i).z() = transform(2,0) * input_V.row(i).x() + transform(2,1) * input_V.row(i).y() + transform(2,2) * input_V.row(i).z() + transform(2,3);
	}
	return V_trans;
}
