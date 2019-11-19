
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "MyViewer.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  MyViewer viewer;
 // viewer.load_mesh_from_file("C:/Users/Owner/Desktop/animation/Ass1/AnimationAssignment1/tutorial/data/cube.obj");
 // viewer.load_mesh_from_file("C:/Users/Owner/Desktop/animation/Ass1/AnimationAssignment1/tutorial/data/bunny.off");
  viewer.load_mesh_from_file("C:/Users/User/Documents/dev/AnimationAssignment1/tutorial/data/sphere.obj");
  viewer.load_mesh_from_file("C:/Users/User/Documents/dev/AnimationAssignment1/tutorial/data/cube.obj");
  viewer.load_mesh_from_file("C:/Users/User/Documents/dev/AnimationAssignment1/tutorial/data/bunny.off");
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
