#pragma once
#include <string>
#include <vector>
#include <io.h>

inline bool find_motions(std::string path, std::vector<std::string>& files, std::string pattern)    
{
	struct _finddata_t file_info;
	std::string temp;
	std::string current_path=path+"\\*";
	long long handle = _findfirst(current_path.c_str(), &file_info);
	if (handle == -1)
		return false;
	do
	{
		temp = file_info.name;
		if(temp.length() > pattern.length() && temp.compare(0, pattern.length(), pattern) == 0) // judge if the string start with pattern
			files.push_back(file_info.name);
	}while(!_findnext(handle, &file_info));
	_findclose(handle);

	if (files.size() == 0)
		return false;
	return true;
}
