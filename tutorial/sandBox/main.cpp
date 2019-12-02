
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "tutorial/sandBox/MyViewer.h"
#include "tutorial/sandBox/MyRenderer.h"
#include "ctime"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  MyRenderer renderer;
  MyViewer viewer;
  viewer.load_configuration();
  int begin = clock();
  viewer.init_simplify_data_structures_list();

  int end = clock();

  std::cout << (double)(end - begin)/ CLOCKS_PER_SEC << std::endl;
  Init(*disp);
  renderer.init(&viewer);
  renderer.my_init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
