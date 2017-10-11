cmake_minimum_required(VERSION 2.8.6)
project(volumetric)
add_compile_options(-std=gnu++11)
add_compile_options(-Wdeprecated-declarations)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)
SET( ENABLE_VMCON3D ON CACHE BOOL "Enable vmcon3d" )

add_subdirectory( fem3d )
add_subdirectory( gui3d )

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

link_directories(./fem3d/)
link_directories(./gui3d/)

file(GLOB vmcon3d_srcs "vmcon3d/*.h" "vmcon3d/*.cpp")

if (ENABLE_VMCON3D)
	project(vmcon3d)
	find_package(DART REQUIRED COMPONENTS utils-urdf gui optimizer-ipopt CONFIG)
	add_compile_options(-DHAVE_CSTDDEF)
	include_directories(${DART_INCLUDE_DIRS})

	add_executable(vmcon3d ${vmcon3d_srcs})
	target_link_libraries(vmcon3d ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${DART_LIBRARIES} fem3d gui3d)
endif()