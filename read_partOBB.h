#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

inline bool read_partOBB(const std::string obj_file_name, std::vector<Eigen::Matrix<double,4,3>> & group_coords)
{
	std::ifstream infile;
	infile.open("../../Data/partOBB/" + obj_file_name + ".txt");
	assert(infile.is_open());
	Eigen::Matrix<double,4,3> temp;
	std::string s;
	group_coords.clear();
	while(!infile.eof()) // 严格要求文件末尾无空行
	{
		infile >> s;
		if (s == "part")
			infile >> s;
		else if (s == "null")
		{
			temp.setZero();
			group_coords.push_back(temp);
		}
		else if (s == "o")
			infile >> temp(0,0) >> temp(0,1) >> temp(0,2);
		else if (s == "x")
			infile >> temp(1,0) >> temp(1,1) >> temp(1,2);
		else if (s == "y")
			infile >> temp(2,0) >> temp(2,1) >> temp(2,2);
		else if (s == "z")
		{
			infile >> temp(3,0) >> temp(3,1) >> temp(3,2);
			group_coords.push_back(temp);
		}
	}

	infile.close();
	return true;
}
