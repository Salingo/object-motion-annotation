#pragma once
#include <Eigen/Core>

struct motion{
	std::string type;
	int movpart, refpart;
	Eigen::Vector2f angrange, disrange;
	Eigen::Vector3f axispos, axisdir;
};
