cmake_minimum_required(VERSION 3.8)
project(rviz_panel_mathplot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
