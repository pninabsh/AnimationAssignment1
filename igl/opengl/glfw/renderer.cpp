#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>

Renderer::Renderer() : selected_core_index(0),
					   next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	highdpi = 1;

	xold = 0;
	yold = 0;
}

IGL_INLINE void Renderer::draw(GLFWwindow *_window)
{
	using namespace std;
	using namespace Eigen;
	window = _window;
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);

	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window, width, height);
		highdpi = highdpi_tmp;
	}

	for (auto &core : core_list)
	{
		core.clear_framebuffers();
	}

	for (auto &core : core_list)
	{
		for (auto &mesh : scn->data_list)
		{
			mesh.slide();
			if (mesh.is_visible/* & core.id*/)
			{
				core.draw(scn->MakeTrans(), mesh);
			}
		}
	}
	//UpdateCamera();

}

void Renderer::SetScene(igl::opengl::glfw::Viewer *viewer)
{
	scn = viewer;
}

Eigen::Vector3d Renderer::getCoordinates(igl::opengl::ViewerData link, bool upLink) {
	Eigen::Vector3d mLink = link.V.colwise().minCoeff();
	Eigen::Vector3d MLink = link.V.colwise().maxCoeff();
	Eigen::Vector4f helpingVector(4);
	if (upLink) {
		helpingVector << (MLink(0) + mLink(0)) / 2, MLink(1), (MLink(2) + mLink(2)) / 2, 1;
	}
	else {
		helpingVector << (MLink(0) + mLink(0)) / 2, mLink(1), (MLink(2) + mLink(2)) / 2, 1;
	}
	Eigen::Vector4f mulVector = link.MakeTrans() * helpingVector;
	return Eigen::Vector3d(mulVector(0), mulVector(1), mulVector(2));
}

void Renderer::UpdateCamera(int right_core) {
	Eigen::Vector3d top = getCoordinates(scn->data_list[14], true);
	//Eigen::Vector4f top_scene = scn->MakeTrans() * scn->data(14).MakeTrans() * Eigen::Vector4f(top(0), top(1), top(2), 1);
	Eigen::Vector4f top_scene = scn->MakeTrans() * Eigen::Vector4f(top(0), top(1), top(2), 1);
	Eigen::Matrix3f rota = scn->data(14).getRotationOfParents();
	// TODO - play wiendth yTheta in order to fix the camera_eye
	core(right_core).camera_up = rota * Eigen::Vector3f(0, 0, 1);
	core(right_core).camera_eye = rota * Eigen::Vector3f(0, -2.09, 0);
	Eigen::Vector3f TR = Eigen::Vector3f(-top_scene(0), -top_scene(1), -top_scene(2));
	Eigen::Vector3f a = (scn->data(14).getRotationOfParents().col(1) * -2);
	TR += a;
	core(right_core).camera_translation = Eigen::Vector3f(TR(0), TR(1), TR(2));
	core(right_core).camera_zoom = 0.5;
}

void Renderer::Locate_Camera(int right_core) {
	Eigen::Vector3d top = getCoordinates(scn->data_list[14], true);
	//Eigen::Vector4f top_scene = scn->MakeTrans() * scn->data(14).MakeTrans() * Eigen::Vector4f(top(0), top(1), top(2), 1);
	Eigen::Vector4f top_scene = scn->MakeTrans() * Eigen::Vector4f(top(0), top(1), top(2), 1);
	Eigen::RowVector3f N = scn->data(14).getRotation() * Eigen::Vector3f(0, 0, 1);
	core(right_core).camera_up = N;
	Eigen::Vector3f E = scn->data(14).getRotation() * Eigen::Vector3f(0, -2.09, 0);
	core(right_core).camera_eye = Eigen::Vector3f(0, 0, 30) + E;
	//Eigen::RowVector4f TR = -(scn->MakeTrans() * scn->data(14).getTranslationOfParents()).col(3).head(3);//A.col(1);
	Eigen::Vector3f TR = Eigen::Vector3f(-top_scene(0), -top_scene(1), -top_scene(2));
	//TR = TR + Eigen::RowVector3f(0, -((0.91) + (9 * 1.6)), 0);
	//Eigen::Vector4f a = (scn->data(14).getRotationOfParents().col(1) * -0.83);
	//TR += a;
	Eigen::Vector3f a = (scn->data(14).getRotationOfParents().col(1) * -2);
	TR += a;
	core(right_core).camera_translation = Eigen::Vector3f(TR(0), TR(1), TR(2));
	core(right_core).camera_zoom = 0.5;

}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer *viewer)
{
	unsigned int left_view, right_view;
	scn = viewer;
	core().init();
	core().align_camera_center(scn->data().V, scn->data().F);
	core().viewport = Eigen::Vector4f(0, 0, 500, 800);
	left_view = core_list[0].id;
	right_view = append_core(Eigen::Vector4f(640, 0, 500, 800));
	viewer->right_core = right_view;
	/*Eigen::RowVector3f N = scn->data(14).getRotation() * Eigen::Vector3f(0, 0, 1);
	core(right_view).camera_up = N;
	Eigen::RowVector3f E = scn->data(0).getRotation() * Eigen::Vector3f(0, 1, 0);
	core(right_view).camera_eye = E;

	Eigen::RowVector3f TR = -scn->data(14).MakeTrans().col(3).head(3); //A.col(1);
	TR = TR + Eigen::RowVector3f(0, -((0.91) + (9 * 1.6)), 0);
	core(right_view).camera_translation = TR;*/
	//UpdateCamera();
	/*Eigen::RowVector3f N = scn->data(14).getRotation() * Eigen::Vector3f(0, 1, 0);
	core(right_view).camera_up = N;*/
	//Eigen::RowVector3f E = scn->data(14).getRotation() * Eigen::Vector3f(0, -1, 0);
	//core(right_view).camera_eye += scn->data(14).getTranslation();
	/*Eigen::Vector3f E = scn->data(14).getTranslation() + Eigen::Vector3f(20 * cos(5), 10, 20 * sin(5));
	core(right_view).camera_eye = E;
	core(right_view).camera_center = scn->data(14).getTranslation();*/
	/*core(right_view).camera_base_translation = Eigen::RowVector3f(0, 0, -5);
	//Eigen::RowVector3f TR = scn->data(14).getTranslation(); //A.col(1);
	Eigen::RowVector3f TR = scn->data(14).getTranslation();
	//TR = TR + Eigen::RowVector3f(0, -((0.91) + (9 * 1.6)), 0)


	core(right_view).camera_translation = TR;*/
	//core(right_view).camera_eye = TR;
	Locate_Camera(right_view);

	for (size_t i = 0; i < scn->data_list.size(); i++) {
		core().toggle(scn->data(i).show_faces);
	}
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{

	if (button == 1) //right
	{
		scn->data().MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0));
		scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0));
		//UpdateCamera();
	}
	else //left
	{
		scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), xrel / 180.0f);
		scn->data().MyRotate(Eigen::Vector3f(0, 0, 1), yrel / 180.0f);
		//UpdateCamera();
	}
}

Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

hitObject Renderer::picking_help(double newx, double newy, int core_id)
{
	hitObject hitObjectCurrent;
	hitObjectCurrent.found = false;
	hitObjectCurrent.distance = 0;

	int fid;
	//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
	Eigen::Vector3f bc;
	double x = newx;
	double y = core(core_id).viewport(3) - newy;
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	igl::look_at(core(core_id).camera_eye, core(core_id).camera_center, core(core_id).camera_up, view);
	view = view * (core(core_id).trackball_angle * Eigen::Scaling(core(core_id).camera_zoom * core(core_id).camera_base_zoom) * Eigen::Translation3f(core(core_id).camera_translation + core(core_id).camera_base_translation)).matrix() * scn->MakeTrans() * scn->data().MakeTrans();
	if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
								 core(core_id).proj, core(core_id).viewport, scn->data().V, scn->data().F, fid, bc))
	{
		//find vertexes - watch igl::ray_mesh_intersect to see how the verixes vere found there and replace f with fid
		Eigen::RowVector3d v0 = scn->data().V.row(scn->data().F(fid, 0)).template cast<double>();
		Eigen::RowVector3d v1 = scn->data().V.row(scn->data().F(fid, 1)).template cast<double>();
		Eigen::RowVector3d v2 = scn->data().V.row(scn->data().F(fid, 2)).template cast<double>();
		// p = alpha1 * p1 + alpha2 * p2 + alpha3 * p3
		Eigen::Vector3f alpha0MultiplyV0(bc(0) * v0(0), bc(0) * v0(1), bc(0) * v0(2));
		Eigen::Vector3f alpha1MultiplyV1(bc(1) * v1(0), bc(1) * v1(1), bc(1) * v1(2));
		Eigen::Vector3f alpha2MultiplyV2(bc(2) * v2(0), bc(2) * v2(1), bc(2) * v2(2));
		Eigen::Vector3f p(alpha0MultiplyV0(0) + alpha1MultiplyV1(0) + alpha2MultiplyV2(0),
			alpha0MultiplyV0(1) + alpha1MultiplyV1(1) + alpha2MultiplyV2(1),
			alpha0MultiplyV0(2) + alpha1MultiplyV1(2) + alpha2MultiplyV2(2));
		//perform transformatiom on P point
		Eigen::Matrix<float, 4, 1> pPointIn4dMatrix = { p(0), p(1), p(2), 1};
		//multiply view * pPointIn4dMatrix in order to get the transformed p point
		Eigen::Matrix<float, 4, 1> resultMatrix = view * pPointIn4dMatrix;
		Eigen::Vector3f resultVector(resultMatrix(0), resultMatrix(1), resultMatrix(2));
		hitObjectCurrent.distance = resultVector.norm();
		hitObjectCurrent.found = true;
		return hitObjectCurrent;
	}

	return hitObjectCurrent;
}

hitObject Renderer::Picking(double newx, double newy){
	hitObject object1 = picking_help(newx, newy, 1);
	//hitObject object2 = picking_help(newx, newy, 2);
	/*if (object1.found) {
		return object1;
	}*/
	return object1;
}

IGL_INLINE void Renderer::resize(GLFWwindow *window, int w, int h)
{
	if (window)
	{
		glfwSetWindowSize(window, w / highdpi, h / highdpi);
	}
	post_resize(window, w, h);
}

IGL_INLINE void Renderer::post_resize(GLFWwindow *window, int w, int h)
{
	if (core_list.size() == 1)
	{
		core().viewport = Eigen::Vector4f(0, 0, w, h);
	}
	else
	{
		// It is up to the user to define the behavior of the post_resize() function
		// when there are multiple viewports (through the `callback_post_resize` callback)
	}
	/*for (unsigned int i = 0; i < scn->plugins.size(); ++i)
	{
		scn->plugins[i]->post_resize(w, h);
	}*/
	if (callback_post_resize)
	{
		callback_post_resize(w, h);
	}
}

IGL_INLINE igl::opengl::ViewerCore &Renderer::core(unsigned core_id /*= 0*/)
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE const igl::opengl::ViewerCore &Renderer::core(unsigned core_id /*= 0*/) const
{
	assert(!core_list.empty() && "core_list should never be empty");
	int core_index;
	if (core_id == 0)
		core_index = selected_core_index;
	else
		core_index = this->core_index(core_id);
	assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
	return core_list[core_index];
}

IGL_INLINE bool Renderer::erase_core(const size_t index)
{
	assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
	//assert(data_list.size() >= 1);
	if (core_list.size() == 1)
	{
		// Cannot remove last viewport
		return false;
	}
	core_list[index].shut(); // does nothing
	core_list.erase(core_list.begin() + index);
	if (selected_core_index >= index && selected_core_index > 0)
	{
		selected_core_index--;
	}
	return true;
}

IGL_INLINE size_t Renderer::core_index(const int id) const
{
	for (size_t i = 0; i < core_list.size(); ++i)
	{
		if (core_list[i].id == id)
			return i;
	}
	return 0;
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
{
	core_list.push_back(core()); // copies the previous active core and only changes the viewport
	core_list.back().viewport = viewport;
	core_list.back().id = next_core_id;
	next_core_id <<= 1;
	if (!append_empty)
	{
		for (auto &data : scn->data_list)
		{
			data.set_visible(true, core_list.back().id);
			//data.copy_options(core(), core_list.back());
		}
	}
	selected_core_index = core_list.size() - 1;
	return core_list.back().id;
}

//IGL_INLINE void Viewer::select_hovered_core()
//{
//	int width_window, height_window = 800;
//   glfwGetFramebufferSize(window, &width_window, &height_window);
//	for (int i = 0; i < core_list.size(); i++)
//	{
//		Eigen::Vector4f viewport = core_list[i].viewport;

//		if ((current_mouse_x > viewport[0]) &&
//			(current_mouse_x < viewport[0] + viewport[2]) &&
//			((height_window - current_mouse_y) > viewport[1]) &&
//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
//		{
//			selected_core_index = i;
//			break;
//		}
//	}
//}