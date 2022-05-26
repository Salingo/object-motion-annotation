#pragma once
#include <Eigen/Core>
#include <vector>
#include <igl/combine.h>

inline bool normalizeOBJ(std::vector<Eigen::MatrixXd> &V, std::vector<Eigen::MatrixXi> &F)
{
	Eigen::MatrixXd V_combine;
	Eigen::MatrixXi F_combine;
	if (V.size() == 1)
	{
		V_combine = V[0];
	}
	else if (V.size() > 1)
	{
		igl::combine<Eigen::MatrixXd, Eigen::MatrixXi>({V[0],V[1]}, {F[0],F[1]}, V_combine, F_combine);
		if(V.size() > 2)
		{
			for (auto i = 2; i < V.size(); i++)
			{
				igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V_combine,V[i]},{F_combine,F[i]},V_combine,F_combine);
			}
		}
	}

	Eigen::Vector3d m = V_combine.colwise().minCoeff();
	Eigen::Vector3d M = V_combine.colwise().maxCoeff();

	double centroid_x = (m(0) + M(0)) / 2;
	double centroid_y = (m(1) + M(1)) / 2;
	double centroid_z = (m(2) + M(2)) / 2;

	double scale_x = M(0) - m(0);
	double scale_y = M(1) - m(1);
	double scale_z = M(2) - m(2);

	double scale = scale_x < scale_y ? (scale_y < scale_z ? scale_z : scale_y) : (scale_x < scale_z ? scale_z : scale_x);
	scale = scale * 1.01;

	for (auto i = 0; i < V.size(); i++)
	{
		for (auto j = 0; j < V[i].rows(); j++)
		{
			V[i](j,0) = (V[i](j,0) - centroid_x) / scale + 0.5;
			V[i](j,1) = (V[i](j,1) - centroid_y) / scale + 0.5;
			V[i](j,2) = (V[i](j,2) - centroid_z) / scale + 0.5;
		}
	}
	return true;
}
