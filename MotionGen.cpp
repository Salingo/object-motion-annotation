#include <igl/combine.h>
#include <igl/random_points_on_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include "read_groupOBJ.h"
#include "get_color.h"
#include "transform.h"
#include "read_partOBB.h"
#include "normalizeOBJ.h"
#include "find_motions.h"
#include "get_volume.h"
#include "motion_struct.h"
#include "io_motion.h"

int main(int argc, char *argv[])
{
	// Init the viewer
	igl::opengl::glfw::Viewer viewer;
	viewer.resize(1200,800);
	// Attach a menu plugin
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);

	// Add content to the default menu window
	menu.callback_draw_viewer_menu = [&]()
	{
		// Viewing options
		if (ImGui::CollapsingHeader("Viewing Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Button("Center object", ImVec2(-1, 0)))
			{
				viewer.core.align_camera_center(viewer.data().V, viewer.data().F);
			}
			if (ImGui::Button("Snap canonical view", ImVec2(-1, 0)))
			{
				viewer.snap_to_canonical_quaternion();
			}

			// Zoom
			ImGui::PushItemWidth(80 * menu.menu_scaling());
			ImGui::DragFloat("Zoom", &(viewer.core.camera_zoom), 0.05f, 0.10f, 20.0f);

			// Select rotation type
			int rotation_type = static_cast<int>(viewer.core.rotation_type);
			static Eigen::Quaternionf trackball_angle = Eigen::Quaternionf::Identity();
			static bool orthographic = true;
			if (ImGui::Combo("Camera Type", &rotation_type, "Trackball\0Two Axes\0002D Mode\0\0"))
			{
			  using RT = igl::opengl::ViewerCore::RotationType;
			  auto new_type = static_cast<RT>(rotation_type);
			  if (new_type != viewer.core.rotation_type)
			  {
				if (new_type == RT::ROTATION_TYPE_NO_ROTATION)
				{
				  trackball_angle = viewer.core.trackball_angle;
				  orthographic = viewer.core.orthographic;
				  viewer.core.trackball_angle = Eigen::Quaternionf::Identity();
				  viewer.core.orthographic = true;
				}
				else if (viewer.core.rotation_type == RT::ROTATION_TYPE_NO_ROTATION)
				{
				  viewer.core.trackball_angle = trackball_angle;
				  viewer.core.orthographic = orthographic;
				}
				viewer.core.set_rotation_type(new_type);
			  }
			}
			// Orthographic view
			ImGui::Checkbox("Orthographic view", &(viewer.core.orthographic));
			ImGui::PopItemWidth();
		}

		// Draw options
		if (ImGui::CollapsingHeader("Draw Options", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::Checkbox("Face-based", &(viewer.data().face_based)))
			{
			  viewer.data().set_face_based(viewer.data().face_based);
			}
			ImGui::Checkbox("Show texture", &(viewer.data().show_texture));
			if (ImGui::Checkbox("Invert normals", &(viewer.data().invert_normals)))
			{
			  viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			}
			ImGui::Checkbox("Show overlay", &(viewer.data().show_overlay));
			ImGui::Checkbox("Show overlay depth", &(viewer.data().show_overlay_depth));
			ImGui::ColorEdit4("Background", viewer.core.background_color.data(),
				ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
			ImGui::ColorEdit4("Line color", viewer.data().line_color.data(),
				ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel);
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.3f);
			ImGui::DragFloat("Shininess", &(viewer.data().shininess), 0.05f, 0.0f, 100.0f);
			ImGui::PopItemWidth();
		}

		// Overlays
		if (ImGui::CollapsingHeader("Overlays", ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::Checkbox("Wireframe", &(viewer.data().show_lines));
			ImGui::Checkbox("Fill", &(viewer.data().show_faces));
			ImGui::Checkbox("Show vertex labels", &(viewer.data().show_vertid));
			ImGui::Checkbox("Show faces labels", &(viewer.data().show_faceid));
		}
	};

	/* -----------------------------------------Draw my window------------------------------------------------ */
	std::vector<Eigen::MatrixXd> V, V_animate;
	std::vector<Eigen::MatrixXi> F, F_animate;
	std::vector<Eigen::Matrix<double,4,3>> group_coords;
	Eigen::MatrixXd V_combine, V_motion;
	Eigen::MatrixXi F_combine, F_motion;
	std::vector<Eigen::MatrixXd> colors;
	motion mo;
	std::vector<std::string> motion_types;
	motion_types.push_back("T");motion_types.push_back("R");motion_types.push_back("TR");
	std::vector<std::string> group_names, motion_names;
	std::string obj_path, obj_name, motion_path;
	std::string point_path = "../../Data/PointCloud";
	std::string label_path = "../../Data/Label";
	int frame_num = 9, frame_id = 0, motion_id = 0, group_id = 0, type_id = 0, mov_id = 0, ref_id = 0, sample_num = 1024;
	bool is_object_loaded = false, is_single_object = false, is_motion_found = false, is_motion_enable = false, is_motion_add = false, is_motion_adjust = false;


	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(950, 0), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(250, 700), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Motion Module", nullptr, ImGuiWindowFlags_NoSavedSettings);

		if (ImGui::Button("Load object", ImVec2(-1, 0)))
		{
			viewer.data().clear();
			motion_names.clear();
			is_single_object = false;
			is_object_loaded = false;
			is_motion_enable = false;
			is_motion_adjust = false;
			is_motion_add = false;

			obj_path = igl::file_dialog_open();
			if (obj_path.length() != 0)
			if (read_groupOBJ(obj_path, V, F, group_names))
			//if (normalizeOBJ(V, F))
			{
				obj_name = obj_path.substr(obj_path.find_last_of("\\") + 1, obj_path.find_last_of(".") - obj_path.find_last_of("\\") - 1);
				std::string obj_path_upper = obj_path.substr(0, obj_path.find_last_of("\\"));
				motion_path = obj_path_upper.substr(0, obj_path_upper.find_last_of("\\")) + "\\motion";
				if (find_motions(motion_path, motion_names, obj_name))
				{
					is_motion_found = true;
				}
				else
				{
					printf("Motion files not found\n");
					is_motion_found = false;
					// Init motion para
					mo.type = "T";
					mo.movpart = 0;
					mo.refpart = 1;
					mo.axispos << 0.5, 0.5, 0.5;
					mo.axisdir << 0, 0, 1;
					mo.angrange << 0, 180;
					mo.disrange << 0, 0.01;
				}
				
				// Combine object parts
				if (V.size() == 1)
				{
					V_combine = V[0];
					F_combine = F[0];
					printf("This is a single object\n");
					is_single_object = true;
				}
				else if (V.size() > 1)
				{
					igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[0],V[1]}, {F[0],F[1]}, V_combine, F_combine);
					if(V.size() > 2)
					{
						for (auto i = 2; i < V.size(); ++i)
							igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V_combine,V[i]}, {F_combine,F[i]}, V_combine, F_combine);
					}
				}

				colors = get_color(F);
				is_object_loaded = true;
				viewer.data().set_mesh(V_combine, F_combine);
				//viewer.snap_to_canonical_quaternion();
				viewer.core.align_camera_center(viewer.data().V, viewer.data().F);
			}
		}
		
		if (is_object_loaded && !is_single_object)
		{
			ImGui::TextColored(ImVec4(1,1,0,1), obj_name.c_str());
			// Display selected part as blue
			ImGui::TextColored(ImVec4(1,1,0,1), "Parts List");
			if (ImGui::Combo("Viusal", &group_id, group_names))
			{
				viewer.data().clear();
				viewer.data().set_mesh(V_combine, F_combine);
				viewer.data().set_colors(colors[group_id]);
			}

			ImGui::TextColored(ImVec4(1,1,0,1), "Motion");
			if (is_motion_found)
			{
				if (ImGui::Combo("Motion File", &motion_id, motion_names))
				{
					read_motion(motion_path+"\\"+motion_names[motion_id], mo);
					is_motion_enable = true;

					igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[mo.refpart],V[mo.movpart]}, {F[mo.refpart],F[mo.movpart]}, V_motion, F_motion);

					// Visual motion axis
					Eigen::Matrix<double, 2, 3> axis;
					axis << mo.axispos(0), mo.axispos(1), mo.axispos(2),
							mo.axispos(0)+0.5*mo.axisdir(0), mo.axispos(1)+0.5*mo.axisdir(1), mo.axispos(2)+0.5*mo.axisdir(2);
					viewer.data().clear();
					viewer.data().set_mesh(V_motion, F_motion);
					viewer.data().add_edges(axis.row(0), axis.row(1), Eigen::RowVector3d(1.0,0.0,0.0).replicate(3,1));
					viewer.data().add_points(axis.row(0), Eigen::RowVector3d(0.0,1.0,0.0));
					viewer.data().add_points(axis.row(1), Eigen::RowVector3d(1.0,0.0,0.0));

					// Create 9 frames of V & F corresponding to current motion
					V_animate.clear();
					F_animate.clear();
					for (int i = 0; i < frame_num; ++i)
					{
						Eigen::MatrixXd V_temp;
						Eigen::MatrixXi F_temp;
						double trans = mo.disrange(0) + i * (mo.disrange(1) - mo.disrange(0)) / (frame_num - 1);
						double angle = mo.angrange(0) + i * (mo.angrange(1) - mo.angrange(0)) / (frame_num - 1);
						Eigen::MatrixXd V_trans = transform(mo.type, mo.axispos, mo.axisdir, trans, angle, V[mo.movpart]);
						igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[mo.refpart],V_trans}, {F[mo.refpart],F[mo.movpart]}, V_temp, F_temp);
						V_animate.push_back(V_temp);
						F_animate.push_back(F_temp);
					}
				}
			}
			else
			{
				is_motion_enable = true;
			}

			// Motion Module
			if (is_motion_enable)
			{
				ImGui::Text("Motion type: %s", mo.type);
				ImGui::Text("Ref part: %s", group_names[mo.refpart]);
				ImGui::Text("Mov part: %s", group_names[mo.movpart]);
				ImGui::TextColored(ImVec4(1,1,0,1), "Animation");
				if (ImGui::SliderInt("Frame", &frame_id, 0, frame_num-1))
				{
					viewer.data().clear();
					viewer.data().set_mesh(V_animate[frame_id], F_animate[frame_id]);
				}

				ImGui::TextColored(ImVec4(1,1,0,1), "Add Motion");
				if (ImGui::Button("Add Motion", ImVec2(-1,0)))
				{
					is_motion_add = true;
					is_motion_adjust = false;
				}
				// Manually add motion
				if (is_motion_add)
				{
					// Adjust axis position, direction and motion range
					if (ImGui::Combo("MotionType", &type_id, motion_types) ||
						ImGui::Combo("Ref part", &ref_id, group_names) ||
						ImGui::Combo("Mov part", &mov_id, group_names) || 
						ImGui::DragFloat("axispos_x", &mo.axispos(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axispos_y", &mo.axispos(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axispos_z", &mo.axispos(2), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_x", &mo.axisdir(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_y", &mo.axisdir(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_z", &mo.axisdir(2), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("ang_min", &mo.angrange(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("ang_max", &mo.angrange(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("dis_min", &mo.disrange(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("dis_max", &mo.disrange(1), 0.01, -1.0, 1.0))
					{
						mo.type = motion_types[type_id];
						mo.refpart = ref_id;
						mo.movpart = mov_id;
						igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[mo.refpart],V[mo.movpart]}, {F[mo.refpart],F[mo.movpart]}, V_motion, F_motion);

						// Visual axis after adjustment
						Eigen::Matrix<double, 2, 3> axis;
						axis << mo.axispos(0), mo.axispos(1), mo.axispos(2),
								mo.axispos(0)+0.5*mo.axisdir(0), mo.axispos(1)+0.5*mo.axisdir(1), mo.axispos(2)+0.5*mo.axisdir(2);
						viewer.data().clear();
						viewer.data().set_mesh(V_motion, F_motion);
						viewer.data().add_edges(axis.row(0), axis.row(1), Eigen::RowVector3d(1.0,0.0,0.0).replicate(3,1));
						viewer.data().add_points(axis.row(0),Eigen::RowVector3d(0.0,1.0,0.0));
						viewer.data().add_points(axis.row(1),Eigen::RowVector3d(1.0,0.0,0.0));

						// Update 9 frames after adjustment
						V_animate.clear();
						F_animate.clear();
						for (int i = 0; i < frame_num; ++i)
						{
							Eigen::MatrixXd V_temp;
							Eigen::MatrixXi F_temp;
							double trans = mo.disrange(0) + i * (mo.disrange(1) - mo.disrange(0)) / (frame_num - 1);
							double angle = mo.angrange(0) + i * (mo.angrange(1) - mo.angrange(0)) / (frame_num - 1);
							Eigen::MatrixXd V_trans = transform(mo.type, mo.axispos, mo.axisdir, trans, angle, V[mo.movpart]);
							igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[mo.refpart],V_trans}, {F[mo.refpart],F[mo.movpart]}, V_temp, F_temp);
							V_animate.push_back(V_temp);
							F_animate.push_back(F_temp);
						}
					}

					if (ImGui::Button("Save Motion", ImVec2(-1,0)))
					{
						std::string motion_path = igl::file_dialog_save();
						save_motion(motion_path, mo);
						is_motion_add = false;
					}
				}

				ImGui::TextColored(ImVec4(1,1,0,1), "Adjust Motion");
				if (ImGui::Button("Adjust Motion", ImVec2(-1,0)))
				{
					is_motion_adjust = true;
					is_motion_add = false;
				}

				// Manually adjust motion
				if (is_motion_adjust)
				{
					// Adjust axis position, direction and motion range
					if (ImGui::DragFloat("axispos_x", &mo.axispos(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axispos_y", &mo.axispos(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axispos_z", &mo.axispos(2), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_x", &mo.axisdir(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_y", &mo.axisdir(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("axisdir_z", &mo.axisdir(2), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("ang_min", &mo.angrange(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("ang_max", &mo.angrange(1), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("dis_min", &mo.disrange(0), 0.01, -1.0, 1.0) ||
						ImGui::DragFloat("dis_max", &mo.disrange(1), 0.01, -1.0, 1.0))
					{
						// Visual axis after adjustment
						Eigen::Matrix<double, 2, 3> axis;
						axis << mo.axispos(0), mo.axispos(1), mo.axispos(2),
								mo.axispos(0)+0.5*mo.axisdir(0), mo.axispos(1)+0.5*mo.axisdir(1), mo.axispos(2)+0.5*mo.axisdir(2);
						viewer.data().clear();
						viewer.data().set_mesh(V_motion, F_motion);
						viewer.data().add_edges(axis.row(0), axis.row(1), Eigen::RowVector3d(1.0,0.0,0.0).replicate(3,1));
						viewer.data().add_points(axis.row(0),Eigen::RowVector3d(0.0,1.0,0.0));
						viewer.data().add_points(axis.row(1),Eigen::RowVector3d(1.0,0.0,0.0));

						// Update 9 frames after adjustment
						V_animate.clear();
						F_animate.clear();
						for (int i = 0; i < frame_num; ++i)
						{
							Eigen::MatrixXd V_temp;
							Eigen::MatrixXi F_temp;
							double trans = mo.disrange(0) + i * (mo.disrange(1) - mo.disrange(0)) / (frame_num - 1);
							double angle = mo.angrange(0) + i * (mo.angrange(1) - mo.angrange(0)) / (frame_num - 1);
							Eigen::MatrixXd V_trans = transform(mo.type, mo.axispos, mo.axisdir, trans, angle, V[mo.movpart]);
							igl::combine<Eigen::MatrixXd,Eigen::MatrixXi>({V[mo.refpart],V_trans}, {F[mo.refpart],F[mo.movpart]}, V_temp, F_temp);
							V_animate.push_back(V_temp);
							F_animate.push_back(F_temp);
						}
					}

					if (ImGui::Button("Save Motion", ImVec2(-1,0)))
					{
						//save_motion("..\\..\\Data\\motion\\" + motion_names[motion_id], mo);
						std::string motion_path = igl::file_dialog_save();
						save_motion(motion_path, mo);
						is_motion_adjust = false;
					}
				}

				// Sample and save 9 pointclouds corresponding to current motion
				ImGui::TextColored(ImVec4(1,1,0,1), "Sample");
				ImGui::InputInt("Points Num", &sample_num);
				ImGui::InputText("Point Path", point_path);
				ImGui::InputText("Label Path", label_path);

				if (ImGui::Button("Sample & Save Points", ImVec2(-1,0)))
				{
					std::ofstream outPoint, outLabel;
					CreateDirectory((point_path + "/" + mo.type).c_str(), NULL);
					CreateDirectory((label_path + "/" + mo.type).c_str(), NULL);

					// Calculate volume of movpart and refpart thus get the number of sample points
					double volume_mov = get_volume(V[mo.movpart], F[mo.movpart]);
					double volume_ref = get_volume(V[mo.refpart], F[mo.refpart]);
					int pointnum_ref = sample_num * volume_ref / (volume_mov + volume_ref);
					if (pointnum_ref < sample_num * 1/12)
						pointnum_ref = sample_num * 1/12;
					else if (pointnum_ref > sample_num * 11/12)
						pointnum_ref = sample_num * 11/12;

					Eigen::SparseMatrix<double> B1, B2;
					Eigen::VectorXi FI1, FI2;
					int num_ref = pointnum_ref;
					int num_mov = sample_num - pointnum_ref;
					Eigen::MatrixXd V_ref = V[mo.refpart], V_mov = V[mo.movpart];
					Eigen::MatrixXi F_ref = F[mo.refpart], F_mov = F[mo.movpart];
					// Sample reference part
					igl::random_points_on_mesh(num_ref, V_ref, F_ref, B1, FI1);
					Eigen::MatrixXd sample_ref = B1 * V[mo.refpart];

					// Sample moving part
					igl::random_points_on_mesh(num_mov, V_mov, F_mov, B2, FI2);
					Eigen::MatrixXd sample_mov = B2 * V[mo.movpart];

					for (int i = 0; i < frame_num; ++i)
					{
						double trans = mo.disrange(0) + i * (mo.disrange(1) - mo.disrange(0)) / (frame_num - 1);
						double angle = mo.angrange(0) + i * (mo.angrange(1) - mo.angrange(0)) / (frame_num - 1);
						Eigen::MatrixXd sample_mov_out = transform(mo.type, mo.axispos, mo.axisdir, trans, angle, sample_mov);

						// Save sample and part label (ref->0 mov->1)
						outPoint.open(point_path + "/" + mo.type + "/" + obj_name + "_u" + std::to_string(motion_id+1) + "_" + std::to_string(i) + ".pts", std::ios::trunc);
						outLabel.open(label_path + "/" + mo.type + "/" + obj_name + "_u" + std::to_string(motion_id+1) + "_" + std::to_string(i) + ".seg", std::ios::trunc);
						for(int j = 0; j < sample_ref.rows(); ++j)
						{
							outPoint << sample_ref.row(j).col(0) << " " << sample_ref.row(j).col(1) << " " << sample_ref.row(j).col(2) << "\n";
							outLabel << 0 << "\n";
						}
						for(int k = 0; k < sample_mov_out.rows(); ++k)
						{
							outPoint << sample_mov_out.row(k).col(0) << " " << sample_mov_out.row(k).col(1) << " " << sample_mov_out.row(k).col(2) << "\n";
							outLabel << 1 << "\n";
						}
						outPoint.close();
						outLabel.close();
						printf("Sample %d saved\n", i);
					}
				}

			}
		}
		ImGui::End();
	};
	viewer.launch();
}
