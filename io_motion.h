#pragma once

inline bool read_motion(std::string name, motion &mo)
{
	std::ifstream infile;
	std::string s;
	infile.open(name);
	assert(infile.is_open());
	infile >> s >> mo.type >> s >> mo.movpart >> s >> mo.refpart;
	infile >> s >> mo.axispos(0) >> mo.axispos(1) >> mo.axispos(2);
	infile >> s >> mo.axisdir(0) >> mo.axisdir(1) >> mo.axisdir(2);
	if (mo.type == "T")
		infile >> s >> mo.disrange(0) >> mo.disrange(1);
	if (mo.type == "R")
		infile >> s >> mo.angrange(0) >> mo.angrange(1);
	if (mo.type == "TR")
	{
		infile >> s >> mo.disrange(0) >> mo.disrange(1);
		infile >> s >> mo.angrange(0) >> mo.angrange(1);		
	}
	printf("Motion file loaded\n");
	infile.close();
	mo.axisdir.normalize();
	return true;
}

inline bool save_motion(std::string name, motion &mo)
{
	std::ofstream outfile;
	outfile.open(name);
	mo.axisdir.normalize();
	assert(outfile.is_open());
	outfile << "type " << mo.type << '\n';
	outfile << "movpart " << mo.movpart << '\n';
	outfile << "refpart " << mo.refpart << '\n';
	outfile << "axispos " << mo.axispos(0) << " " << mo.axispos(1) << " " << mo.axispos(2) << '\n';
	outfile << "axisdir " << mo.axisdir(0) << " " << mo.axisdir(1) << " " << mo.axisdir(2) << '\n';
	if (mo.type == "T")
		outfile << "disrange " << mo.disrange(0) << " " << mo.disrange(1) << '\n';
	else if (mo.type == "R")
		outfile << "angrange " << mo.angrange(0) << " " << mo.angrange(1) << '\n';
	else if (mo.type == "TR")
	{
		outfile << "disrange " << mo.disrange(0) << " " << mo.disrange(1) << '\n';
		outfile << "angrange " << mo.angrange(0) << " " << mo.angrange(1) << '\n';
	}
	printf("Motion file saved\n");
	outfile.close();
	return true;
}