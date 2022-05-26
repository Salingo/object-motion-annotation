#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>
#include <cstdio>
#include <sstream>
#include <iterator>
#include <string>
#include <igl/list_to_matrix.h>
#include <igl/max_size.h>
#include <igl/min_size.h>

template <typename Scalar, typename Index>
bool read_groupOBJ(
  const std::string obj_file_name,
  std::vector<std::vector<std::vector<Scalar>>> & V,
  std::vector<std::vector<std::vector<Index>>> & F,
  std::vector<std::string> & group_name)
{
  // Open file, and check for error
  FILE * obj_file = fopen(obj_file_name.c_str(),"r");
  if(NULL==obj_file)
  {
    fprintf(stderr,"IOError: %s could not be opened...\n", obj_file_name.c_str());
    return false;
  }
  // File open was successful so clear outputs
  V.clear();
  F.clear();
  group_name.clear();

  int group_offset = 0;
  bool is_ReadingGroup = false;

  std::vector<std::vector<Scalar>> V_group;
  std::vector<std::vector<Index>> F_group;

  // variables and constants to assist parsing the .obj file
  // Constant strings to compare against
  std::string v("v");
  std::string f("f");
  std::string g("g");
  std::string tic_tac_toe("#");
#  define LINE_MAX 2048

  char line[LINE_MAX];
  int line_no = 1;
  while (fgets(line, LINE_MAX, obj_file) != NULL)
  {
    char type[LINE_MAX];
    // Read first word containing type
    if(sscanf(line, "%s",type) == 1)
    {
      // Get pointer to rest of line right after type
      char * l = &line[strlen(type)];
      if(is_ReadingGroup && type == v)
      {
	      is_ReadingGroup = false;
		  group_offset += V_group.size();
		  V.push_back(V_group);
		  F.push_back(F_group);
		  V_group.clear();
		  F_group.clear();
      }
      if(type == v)
      {
        std::istringstream ls(&line[1]);
        std::vector<Scalar> vertex{ std::istream_iterator<Scalar>(ls), std::istream_iterator<Scalar>() };

        if (vertex.size() < 3)
        {
          fprintf(stderr,
                  "Error: readGroupOBJ() vertex on line %d should have at least 3 coordinates",
                  line_no);
          fclose(obj_file);
          return false;
        }
        V_group.push_back(vertex);
      }else if(type == f)
      {
        const auto & shift = [&V_group](const int i)->int
        {
          return i<0 ? i+V_group.size() : i-1;
        };
        std::vector<Index > f;
        // Read each "word" after type
        char word[IGL_LINE_MAX];
        int offset;
        while(sscanf(l,"%s%n",word,&offset) == 1)
        {
          // adjust offset
          l += offset;
          // Process word
          long int i;
          if(sscanf(word,"%ld",&i) == 1)
          {
            f.push_back(shift(i-group_offset));
          }else
          {
            fprintf(stderr,
                    "Error: readGroupOBJ() face on line %d has invalid element format\n",
                    line_no);
            fclose(obj_file);
            return false;
          }
        }
        if(f.size()>0)
        {
          F_group.push_back(f);
        }else
        {
          fprintf(stderr,
                  "Error: readGroupOBJ() face on line %d has invalid format\n", line_no);
          fclose(obj_file);
          return false;
        }
      }else if(type == g)
      {
	      is_ReadingGroup = true;
		  group_name.push_back(l);
      }else if(strlen(type) >= 1 && (type[0] == '#' ||
            type[0] == 's'  ||
            strcmp("usemtl",type)==0 ||
            strcmp("mtllib",type)==0))
      {
        //ignore comments or other shit
      }else
      {
        //ignore any other lines
        //fprintf(stderr,
        //        "Warning: readGroupOBJ() ignored non-comment line %d:\n  %s",
        //        line_no,
        //        line);
      }
    }else
    {
      // ignore empty line
    }
    line_no++;
  }
  // read the last group
  if(V_group.size() && F_group.size())
  {
	V.push_back(V_group);
	F.push_back(F_group);
  }

  if(group_name.size() == 0)
  {
	group_name.push_back(obj_file_name.substr(obj_file_name.find_last_of("/") + 1, obj_file_name.find_last_of(".") - obj_file_name.find_last_of("/") - 1));
  }
  fclose(obj_file);
  assert(V.size() == group_name.size());
  return true;
}


template <typename Scalar, typename Index>
bool read_groupOBJ(
  const std::string str,
  std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>& V,
  std::vector<Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic>>& F,
  std::vector<std::string>& group_name)
{
  std::vector<std::vector<std::vector<double>>> vV;
  std::vector<std::vector<std::vector<int>>> vF;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> V_group;
  Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic> F_group;

  V.clear();
  F.clear();
  group_name.clear();
  bool success = read_groupOBJ(str,vV,vF,group_name);
  if(!success)
  {
    return false;
  }
  for(int i = 0; i < vV.size(); i++)
  {
  	bool V_rect = igl::list_to_matrix(vV[i],V_group);
    const char * format = "Failed to cast %s to matrix: min (%d) != max (%d)\n";
    if(!V_rect)
    {
      printf(format,"V",igl::min_size(vV),igl::max_size(vV));
      return false;
    }
	V.push_back(V_group);

    bool F_rect = igl::list_to_matrix(vF[i],F_group);
    if(!F_rect)
    {
      printf(format,"F",igl::min_size(vF),igl::max_size(vF));
      return false;
    }
	F.push_back(F_group);
  }
  return true;
}
