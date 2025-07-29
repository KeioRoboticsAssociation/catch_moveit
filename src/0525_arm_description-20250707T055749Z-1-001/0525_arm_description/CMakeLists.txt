################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(0525_arm_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

#install(
#    DIRECTORY config/ # ソースのconfigディレクトリ
#   DESTINATION share/${PROJECT_NAME}/config # インストール先の share/<package_name>/config/
#    PATTERN "*.srdf" # SRDFファイルのみをコピーする場合 (推奨)
#    PATTERN "*.yaml" # configディレクトリに他の.yamlファイルがある場合
#    PATTERN ".gitkeep" EXCLUDE # .gitkeepを除外
#)

################################################################################
# Macro for ament package
################################################################################
ament_package()
