#pragma once
#include <Eigen/Core>

// Get color for each part
inline std::vector<Eigen::MatrixXd> get_color(std::vector<Eigen::MatrixXi> F)
{	
	Eigen::MatrixXd C;
	std::vector<Eigen::MatrixXd> colors;
	int allrows = 0;
	int lastrow = 0;

	for (auto i = 0; i < F.size(); ++i)
	  allrows += F[i].rows();

	for (auto k = 0; k < F.size(); ++k)
	{
		C = Eigen::RowVector3d(1.0,0.7,0.2).replicate(allrows,1);
		for (auto j = lastrow; j < lastrow + F[k].rows(); ++j)
			C.row(j) = Eigen::RowVector3d(0.2,0.3,0.8);
		lastrow += F[k].rows();
		colors.push_back(C);
	}
	return colors;
}
