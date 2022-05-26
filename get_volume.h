#pragma once
#include <igl/volume.h>

inline double get_volume(Eigen::MatrixXd V, Eigen::MatrixXi F)
{
	Eigen::MatrixXd V2(V.rows() + 1, V.cols());
	V2.topRows(V.rows()) = V;
	V2.bottomRows(1).setZero();
	Eigen::MatrixXi T(F.rows(), 4);
	T.leftCols(3) = F;
	T.rightCols(1).setConstant(V.rows());
	Eigen::VectorXd vol;
	igl::volume(V2, T, vol);
	return std::abs(vol.sum());
}

