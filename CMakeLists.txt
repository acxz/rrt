# @author: Akash Patel
# @date: 5/21/19

cmake_minimum_required(VERSION 3.14)

project(rrt)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include the project files
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
                    ${Eigen_INLCUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

# Collect the source files
file(GLOB exe "exe/*.cpp")
list(SORT exe)
foreach(exe_file ${exe})
	get_filename_component(exe_base ${exe_file} NAME_WE)
  message(STATUS "Adding exe ${exe_file} with base name ${exe_base}")

  add_executable(${exe_base} ${exe_file})

  target_link_libraries(${exe_base} ${OMPL_LIBRARIES} Eigen3::Eigen -lboost_system)
  add_custom_target(${exe_base}.run ${exe_base} ${ARGN})
endforeach(exe_file)
