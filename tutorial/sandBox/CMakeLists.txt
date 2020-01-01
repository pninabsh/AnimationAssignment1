get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configuration.txt
    ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)

add_executable(${PROJECT_NAME}_bin main.cpp inputManager.h MyRenderer.h MyRenderer.cpp MyViewer.h MyViewer.cpp)
target_link_libraries(${PROJECT_NAME}_bin igl::core igl::opengl igl::opengl_glfw)
