cmake_minimum_required(VERSION 3.5)
project(gofa_viz)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install all resources (meshes, configs, etc.)
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
