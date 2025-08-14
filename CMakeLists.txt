cmake_minimum_required(VERSION 3.8)
project(rviz_panel_mathplot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## mathplot
# mathplot uses c++-20 language features
set(CMAKE_CXX_STANDARD 20)

# Tell the program where the mathplot fonts are, to compile them into the binary
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMPLOT_FONTS_DIR=\"\\\"${PROJECT_SOURCE_DIR}/mathplot/fonts\\\"\"")

# Find the libraries which will be needed by mathplot
set(OpenGL_GL_PREFERENCE "GLVND") # Following `cmake --help-policy CMP0072`
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(Freetype REQUIRED)
find_package(nlohmann_json REQUIRED)
#find_package(Armadillo) # Optional
#find_package(HDF5)      # Optional

set(MPLOT_INCLUDELIST ${OPENGL_INCLUDE_DIR} ${GLFW3_INCLUDE_DIR} ${FREETYPE_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/maths ${PROJECT_SOURCE_DIR}/mathplot)
## end mathplot

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/rviz_panel_mathplot/mathplot_panel.hpp
)

add_library(mathplot_panel src/mathplot_panel.cpp ${MOC_FILES})
target_include_directories(mathplot_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mathplot_panel PRIVATE ${MPLOT_INCLUDELIST})

ament_target_dependencies(mathplot_panel
  pluginlib
  rviz_common
)
install(TARGETS mathplot_panel
        EXPORT export_rviz_panel_mathplot
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/DemoPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_panel_mathplot)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
