

#include <chrono>
#include <thread>

#include "../gl.h"
//#include "<GLFW/glfw3.h>"
#include "Display.h"
#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include "tutorial/sandBox/IK_solver.h"
#include "tutorial/sandBox/MyRenderer.h"
#include "../../ImGui/imgui.h"
#include <external\imgui\imgui.h>
#include <igl/get_seconds.h>
#include <igl/PI.h>
//#include "../../imgui/"
//#include "tutorial/sandBox/Imgui/imgui.h"
//#include "tutorial/sandBox/Imgui/imgui_impl_glfw_gl3.h"
//#include "Imgui/"
#include "imgui/ImGuiMenu.h"

static void glfw_error_callback(int error, const char *description)
{
	fputs(description, stderr);
}

void drawText(const char *text, int length, int x, int y)
{
	//glMatrixMode(GL_PROJECTION);
}

Display::Display(int windowWidth, int windowHeight, const std::string &title)
{
	bool resizable = true, fullscreen = false;
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
	{
		exit(EXIT_FAILURE);
	}
	glfwWindowHint(GLFW_SAMPLES, 8);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	//#ifdef __APPLE__
	//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	//#endif
	//		if (fullscreen)
	//		{
	//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
	//			windowWidth = mode->width;
	//			windowHeight = mode->height;
	//		}
	//		else
	//		{
	// Set default windows width
	//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
	//	windowWidth = renderer->core().viewport[2];
	//else
	//	if (windowWidth <= 0)
	//	windowWidth = 1280;
	//// Set default windows height
	//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
	//	windowHeight = renderer->core().viewport[3];
	//else if (windowHeight <= 0)
	//	windowHeight = 800;
	//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
	//		}
	window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);
	// Load OpenGL and its extensions
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		printf("Failed to load OpenGL and its extensions\n");
		exit(EXIT_FAILURE);
	}
#if defined(DEBUG) || defined(_DEBUG)
	printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
	int major, minor, rev;
	major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
	minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
	rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
	printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
	printf("Supported OpenGL is %s\n", (const char *)glGetString(GL_VERSION));
	printf("Supported GLSL is %s\n", (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION));
#endif
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	//Tamir: changes from here
	// Initialize FormScreen
	// __viewer = this;
	// Register callbacks
	//glfwSetKeyCallback(window, glfw_key_callback);
	//glfwSetCursorPosCallback(window,glfw_mouse_move);
	//glfwSetScrollCallback(window, glfw_mouse_scroll);
	//glfwSetMouseButtonCallback(window, glfw_mouse_press);
	//glfwSetWindowSizeCallback(window,glfw_window_size);

	//glfwSetCharModsCallback(window,glfw_char_mods_callback);
	//glfwSetDropCallback(window,glfw_drop_callback);
	// Handle retina displays (windows and mac)
	//int width, height;
	//glfwGetFramebufferSize(window, &width, &height);
	//int width_window, height_window;
	//glfwGetWindowSize(window, &width_window, &height_window);
	//highdpi = windowWidth/width_window;

	//glfw_window_size(window,width_window,height_window);
	//opengl.init();
	//		core().align_camera_center(data().V, data().F);
	// Initialize IGL viewer
	//		init();
}

bool Display::launch_rendering(bool loop)
{
	// glfwMakeContextCurrent(window);
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer *renderer = (Renderer *)glfwGetWindowUserPointer(window);
	//renderer->draw(window);
	MyViewer viewer;
	/*ImGui::CreateContext();
	ImGui_ImplGlfwGL3_Init(window, true);
	ImGui::StyleColorsDark();*/
	//ImGui::CreateContext();
	//igl::opengl::glfw::imgui::ImGuiMenu menu;
	/*menu.init(renderer);
	menu.pre_draw();*/
	/*viewer.plugins.push_back(&menu);*/

	// Customize the menu
	//double doubleVariable = 0.1f; // Shared between two menus
	/*menu.init(renderer);
	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
		ImGui::Begin(
			"New Window", nullptr,
			ImGuiWindowFlags_NoSavedSettings
		);

		// Expose the same variable directly ...
		ImGui::PushItemWidth(-80);
		ImGui::DragScalar("double", ImGuiDataType_Double, &doubleVariable, 0.1, 0, 0, "%.4f");
		ImGui::PopItemWidth();

		static std::string str = "bunny";
		//ImGui::InputText("Name", str);

		ImGui::End();
	};
	//menu.draw_custom_window();
	menu.pre_draw();
	//menu.draw_menu();
	menu.post_draw();
	viewer.plugins.push_back(&menu);*/
	/*menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
		ImGui::Begin(
			"New Window", nullptr,
			ImGuiWindowFlags_NoSavedSettings
		);

		// Expose the same variable directly ...
		ImGui::PushItemWidth(-80);
		ImGui::DragScalar("double", ImGuiDataType_Double, &doubleVariable, 0.1, 0, 0, "%.4f");
		ImGui::PopItemWidth();

		static std::string str = "bunny";
		//ImGui::InputText("Name", str);

		ImGui::End();
	};*/
	//menu.draw_custom_window();
	/*menu.init(renderer);
	menu.pre_draw();*/
	//igl::opengl::glfw::imgui::ImGuiMenu menu;
	/*menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		menu.draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
			//ImGui::InputFloat("float", &doubleVariable, 0, 0, 3);

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable))
			{
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int*)(&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices))
			{
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int)choices.size())
			{
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			//ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
			{
				std::cout << "Hello\n";
			}
		}
	};*/
	//menu.init(renderer);
	//menu.pre_draw();
	//menu.post_draw();

	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	igl::opengl::glfw::Viewer *scn = renderer->scn;
	std::stringstream l1;
	l1 << "Score 20";
	scn->data().add_label(Eigen::Vector3d(5, 2, -2), l1.str());

	//viewer.plugins.push_back(&menu);

	while (!glfwWindowShouldClose(window))
	{
		MyViewer viewer;
		//scn->data().add_label(Eigen::Vector3d(5, 2, -2), "Score: ");
		/*ImGui::CreateContext();
		ImGui_ImplGlfwGL3_Init(window, true);
		ImGui::StyleColorsDark();*/
		//ImGui::CreateContext();
		//igl::opengl::glfw::imgui::ImGuiMenu menu;
		//glfwPollEvents();
		//glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
		//glClear(GL_COLOR_BUFFER_BIT);

		// Customize the menu
		double doubleVariable = 0.1f; // Shared between two menus

		//ImGui_ImplGlfwGL3_NewFrame();
		double tic = igl::get_seconds();
		renderer->UpdateCamera(scn->right_core);
		renderer->draw(window);

		for (int i = 0; i <= 4; i++)
		{
			double t = 0.25 * igl::get_seconds();
			scn->data_list[i].MyRotate2(Eigen::AngleAxisf(t * 2. * igl::PI, Eigen::Vector3f(0, 1, 0)));
			scn->data_list[i].MyTranslate(Eigen::Vector3f(0, 0.125 * cos(2. * igl::PI * t), 0));
		}
		//float doubleVariable = 0.1f; // Shared between two menus

		//menu.callback_draw_custom_window = [&]() {};
		/*menu.callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			menu.draw_viewer_menu();

			// Add new group
			if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
			{
				// Expose variable directly ...
				//ImGui::InputFloat("float", &doubleVariable, 0, 0, 3);

				// ... or using a custom callback
				static bool boolVariable = true;
				if (ImGui::Checkbox("bool", &boolVariable))
				{
					// do something
					std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
				}

				// Expose an enumeration type
				enum Orientation { Up = 0, Down, Left, Right };
				static Orientation dir = Up;
				ImGui::Combo("Direction", (int*)(&dir), "Up\0Down\0Left\0Right\0\0");

				// We can also use a std::vector<std::string> defined dynamically
				static int num_choices = 3;
				static std::vector<std::string> choices;
				static int idx_choice = 0;
				if (ImGui::InputInt("Num letters", &num_choices))
				{
					num_choices = std::max(1, std::min(26, num_choices));
				}
				if (num_choices != (int)choices.size())
				{
					choices.resize(num_choices);
					for (int i = 0; i < num_choices; ++i)
						choices[i] = std::string(1, 'A' + i);
					if (idx_choice >= num_choices)
						idx_choice = num_choices - 1;
				}
				//ImGui::Combo("Letter", &idx_choice, choices);

				// Add a button
				if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
				{
					std::cout << "Hello\n";
				}
			}
		};*/
		// Draw additional windows
		/*menu.callback_draw_custom_window = [&]()
		{
			// Define next window position + size
			ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
			ImGui::Begin(
				"New Window", nullptr,
				ImGuiWindowFlags_NoSavedSettings
			);

			// Expose the same variable directly ...
			ImGui::PushItemWidth(-80);
			ImGui::DragScalar("double", ImGuiDataType_Double, &doubleVariable, 0.1, 0, 0, "%.4f");
			ImGui::PopItemWidth();

			static std::string str = "bunny";
			//ImGui::InputText("Name", str);

			ImGui::End();
		};*/
		//menu.init(renderer);
		/*menu.callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			menu.draw_viewer_menu();

			// Add new group
			if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
			{
				// Expose variable directly ...
				//ImGui::InputFloat("float", &doubleVariable, 0, 0, 3);

				// ... or using a custom callback
				static bool boolVariable = true;
				if (ImGui::Checkbox("bool", &boolVariable))
				{
					// do something
					std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
				}

				// Expose an enumeration type
				enum Orientation { Up = 0, Down, Left, Right };
				static Orientation dir = Up;
				ImGui::Combo("Direction", (int*)(&dir), "Up\0Down\0Left\0Right\0\0");

				// We can also use a std::vector<std::string> defined dynamically
				static int num_choices = 3;
				static std::vector<std::string> choices;
				static int idx_choice = 0;
				if (ImGui::InputInt("Num letters", &num_choices))
				{
					num_choices = std::max(1, std::min(26, num_choices));
				}
				if (num_choices != (int)choices.size())
				{
					choices.resize(num_choices);
					for (int i = 0; i < num_choices; ++i)
						choices[i] = std::string(1, 'A' + i);
					if (idx_choice >= num_choices)
						idx_choice = num_choices - 1;
				}
				//ImGui::Combo("Letter", &idx_choice, choices);

				// Add a button
				if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
				{
					std::cout << "Hello\n";
				}
			}
		};*/
		//menu.callback_draw_viewer_window = []() {};
		//menu.draw_labels_window();
		//viewer.plugins.push_back(&menu);
		//menu.init(renderer);
		//menu.draw_example();
		//menu.pre_draw();

		//menu.post_draw();
		//
		//igl::opengl::glfw::imgui::ImGuiMenu menu;

		//viewer.data().add_label(Eigen::Vector3d(0, 0, 0), "Score: ");
		//menu.draw_text(Eigen::Vector3d(10, 10, 0), Eigen::Vector3d(0, 1, 0), "Hi", Eigen::Vector4f(0, 0, 0.043, 0));
		//viewer.plugins.push_back(&menu);

		//ImGui::Render();
		//ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{ //motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

		MyRenderer *rndr = (MyRenderer *)glfwGetWindowUserPointer(window);
		MyViewer *scn = rndr->GetMyScene();
		if (getIsAnimating())
		{
			ccd_step(scn, rndr->GetMyScene()->selected_data_index);
			//rndr->UpdateCamera(scn->right_core);
		}

		if (timer.getElapsedTimeInSec() > 12 && !scn->is_waiting_for_user())
		{
			scn->end_level();
			renderer->core(scn->right_core).background_color.setConstant(0);
			renderer->core(1).background_color.setConstant(0);
		}

		//menu.shutdown();
		//ImGui_ImplGlfwGL3_Shutdown();
		//ImGui::DestroyContext();
#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack)
		{
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void (*keyCallback)(GLFWwindow *, int, int, int, int))
{
	glfwSetKeyCallback(window, (void (*)(GLFWwindow *, int, int, int, int))keyCallback); //{
}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow *, int, int, int), void (*scrollfun)(GLFWwindow *, double, double), void (*cursorposfun)(GLFWwindow *, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow *, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void *userPointer)
{

	glfwSetWindowUserPointer(window, userPointer);
}

void *Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}
