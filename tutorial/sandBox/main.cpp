
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "tutorial/sandBox/MyViewer.h"
#include "tutorial/sandBox/MyRenderer.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  MyRenderer renderer;
  MyViewer viewer;
  viewer.load_configuration();
  viewer.init_simplify_data_structures_list();
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
