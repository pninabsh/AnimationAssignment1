
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "tutorial/sandBox/MyViewer.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  MyViewer viewer;
  viewer.load_configuration();
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
