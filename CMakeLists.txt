cmake_minimum_required(VERSION 2.9)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ")

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
# change yolo
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_definitions(-w)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(OpenCV_DIR "/usr/local/opencv341/share/OpenCV")
find_package(OpenCV EXACT 3.4.1 QUIET)

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
# change yolo
set(Torch_DIR /media/lzh/新加卷/Ubuntu1804_Copy/Libtorch/libtorch_line/share/cmake/Torch)
find_package(Torch PATHS ${Torch_DIR} NO_DEFAULT REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

#myplan

# find openmp
find_package(OpenMP)
if (OPENMP_FOUND)
   message("-- OpenMP found")
   set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
   set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find CGAL
set(CMAKE_BUILD_TYPE_BACKUP ${CMAKE_BUILD_TYPE})
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BACKUP})


# Find Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})
# BLAS and LAPACK
find_package(LAPACK REQUIRED)
link_directories(${LAPACK_LIBRARY_DIR})

# change 添加sophus
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/include/
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp
${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp/build
${PROJECT_SOURCE_DIR}/Thirdparty/EDTest
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/gridStructure.cpp
src/LineExtractor.cc
src/LineIterator.cpp
src/LineMatcher.cpp
src/MapLine.cc
        src/YOLOV5.cc
        src/ProbabilityMapping.cc
        src/Modeler.cc
        src/CARV/Exception.cpp
        src/CARV/FreespaceDelaunayAlgorithm.cc
        src/CARV/GraphWrapper_Boost.cc
        src/CARV/lovimath.cc
        src/CARV/Matrix.cc
        src/CARV/SFMTranscript.cpp
        src/CARV/SFMTranscriptInterface_Delaunay.cpp
        src/CARV/SFMTranscriptInterface_ORBSLAM.cpp
        src/CARV/StringFunctions.cpp
        src/detect_3d_cuboid/box_proposal_detail.cpp
        src/detect_3d_cuboid/matrix_utils.cpp
        src/detect_3d_cuboid/object_3d_util.cpp
        src/Object.cc
        src/LineDetector.cc

        src/planar_mapping_module.cc
        src/landmark_plane.cc


        include/GCRANSAC/estimator/estimator.h
        include/GCRANSAC/estimator/homography_estimator.h
        include/GCRANSAC/estimator/rigid_transformation_estimator.h
        include/GCRANSAC/estimator/linear_model_estimator.h
        include/GCRANSAC/pearl/block.h
        include/GCRANSAC/pearl/energy.h
        include/GCRANSAC/pearl/GCoptimization.cpp
        include/GCRANSAC/pearl/GCoptimization.h
        include/GCRANSAC/pearl/graph.cpp
        include/GCRANSAC/pearl/graph.h
        include/GCRANSAC/pearl/LinkedBlockList.cpp
        include/GCRANSAC/pearl/LinkedBlockList.h
        include/GCRANSAC/pearl/maxflow.cpp
        include/GCRANSAC/solver/solver_engine.h
        include/GCRANSAC/solver/solver_homography_four_point.h
        include/GCRANSAC/solver/solver_linear_model.h
        include/GCRANSAC/solver/solver_rigid_transformation_svd.h
        include/GCRANSAC/flann_neighborhood_graph.h
        include/GCRANSAC/GCRANSAC.h
        include/GCRANSAC/grid_neighborhood_graph.h
        include/GCRANSAC/model.h
        include/GCRANSAC/neighborhood_graph.h
        include/GCRANSAC/preemption_empty.h
        include/GCRANSAC/preemption_sprt.h
        include/GCRANSAC/sampler.h
        include/GCRANSAC/scoring_function.h
        include/GCRANSAC/settings.h
        include/GCRANSAC/statistics.h
        include/GCRANSAC/types.h
        include/GCRANSAC/uniform_sampler.h
        include/GCRANSAC/uniform_random_generator.h

        include/landmark_plane.h
        include/planar_mapping_module.h
         include/types.h
         include/LineDetector.h
         include/EdgeMap.h
        include/Object.h
        include/isolation_forest.h
         include/detect_3d_cuboid/detect_3d_cuboid.h
         include/detect_3d_cuboid/matrix_utils.h
         include/detect_3d_cuboid/object_3d_util.h
        include/CARV/Exception.h
        include/CARV/FreespaceDelaunayAlgorithm.h
        include/CARV/GraphWrapper_Boost.h
        include/CARV/lovimath.h
        include/CARV/Matrix.h
        include/CARV/SFMTranscript.h
        include/CARV/SFMTranscriptInterface_Delaunay.h
        include/CARV/SFMTranscriptInterface_ORBSLAM.h
        include/CARV/StringFunctions.h

include/ProbabilityMapping.h
include/Modeler.h
include/gridStructure.h
include/LineExtractor.h
include/LineIterator.h
include/LineMatcher.h
include/MapLine.h

include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/PnPsolver.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Initializer.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/TwoViewReconstruction.h
src/TwoViewReconstruction.cc

        include/YOLOV5.h
        include/utils.h
        )

add_subdirectory(Thirdparty/g2o)
#change 添加torch
#myline 添加line
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${TORCH_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/line_descriptor/lib/liblinedesc.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/Thirdparty/EDLines/EDLinesLib.a
${PROJECT_SOURCE_DIR}/Thirdparty/EDTest/EDLib.a
${PROJECT_SOURCE_DIR}/Thirdparty/Line3Dpp/build/libline3Dpp.so
boost_system boost_filesystem boost_serialization
lapack blas
#-lboost_serialization
#-lcrypto
)


# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

add_executable(mono_UMA
Examples/Monocular/mono_UMA.cc)
target_link_libraries(mono_UMA ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

add_executable(mono_inertial_UMA
Examples/Monocular-Inertial/mono_inertial_UMA.cc)
target_link_libraries(mono_inertial_UMA ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Line-Inertial)

add_executable(stereo_line_inertial_euroc
Examples/Stereo-Line-Inertial/stereo_line_inertial_euroc.cc)
target_link_libraries(stereo_line_inertial_euroc ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Line)

add_executable(stereo_line_euroc
Examples/Stereo-Line/stereo_line_euroc.cc)
target_link_libraries(stereo_line_euroc ${PROJECT_NAME})

add_executable(stereo_line_UMA
Examples/Stereo-Line/stereo_line_UMA.cc)
target_link_libraries(stereo_line_UMA ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Kitti)

add_executable(stereo_line_kitti Examples/Stereo-Kitti/stereo_kitti.cc)
target_link_libraries(stereo_line_kitti ${PROJECT_NAME})
