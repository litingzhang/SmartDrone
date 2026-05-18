set(SMART_DRONE_ORB_MODULE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(SMART_DRONE_ORB_SLAM3_LIB_DIR "${CMAKE_BINARY_DIR}/orb_slam3/lib")

find_package(OpenCV 4.2 REQUIRED COMPONENTS core features2d highgui imgproc)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Eigen3 3.1.0 REQUIRED)

set(SMART_DRONE_ORB_MODULE_INCLUDE_DIRS
  "${SMART_DRONE_REPO_ROOT}/src/native"
  "${SMART_DRONE_ORB_MODULE_ROOT}"
  "${SMART_DRONE_ORB_MODULE_ROOT}/camera"
  "${SMART_DRONE_ORB_MODULE_ROOT}/camera/CameraModels"
  "${SMART_DRONE_ORB_MODULE_ROOT}/common"
  "${SMART_DRONE_ORB_MODULE_ROOT}/config"
  "${SMART_DRONE_ORB_MODULE_ROOT}/features"
  "${SMART_DRONE_ORB_MODULE_ROOT}/frame"
  "${SMART_DRONE_ORB_MODULE_ROOT}/geometry"
  "${SMART_DRONE_ORB_MODULE_ROOT}/graph_optimization"
  "${SMART_DRONE_ORB_MODULE_ROOT}/loop_closing"
  "${SMART_DRONE_ORB_MODULE_ROOT}/map"
  "${SMART_DRONE_ORB_MODULE_ROOT}/mapping"
  "${SMART_DRONE_ORB_MODULE_ROOT}/place_recognition"
  "${SMART_DRONE_ORB_MODULE_ROOT}/system"
  "${SMART_DRONE_ORB_MODULE_ROOT}/tracking"
)

set(SMART_DRONE_ORB_THIRD_PARTY_INCLUDE_DIRS
  "${SMART_DRONE_DBOW2_ROOT}"
  "${SMART_DRONE_G2O_ROOT}"
  "${SMART_DRONE_SOPHUS_ROOT}"
)

function(configure_orb_component target_name)
  target_include_directories(${target_name} PUBLIC
    ${SMART_DRONE_ORB_MODULE_INCLUDE_DIRS}
    ${SMART_DRONE_ORB_THIRD_PARTY_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${target_name} PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
  )
  target_compile_options(${target_name} PRIVATE -O3 -w)
  set_target_properties(${target_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

set(SMART_DRONE_ORB_DBOW2_SOURCES
  "${SMART_DRONE_DBOW2_ROOT}/DBoW2/BowVector.cpp"
  "${SMART_DRONE_DBOW2_ROOT}/DBoW2/FORB.cpp"
  "${SMART_DRONE_DBOW2_ROOT}/DBoW2/FeatureVector.cpp"
  "${SMART_DRONE_DBOW2_ROOT}/DBoW2/ScoringObject.cpp"
  "${SMART_DRONE_DBOW2_ROOT}/DUtils/Random.cpp"
  "${SMART_DRONE_DBOW2_ROOT}/DUtils/Timestamp.cpp"
)

add_library(DBoW2 SHARED ${SMART_DRONE_ORB_DBOW2_SOURCES})
target_include_directories(DBoW2 PUBLIC
  "${SMART_DRONE_DBOW2_ROOT}"
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS})
target_compile_options(DBoW2 PRIVATE -O3 -w)
set_target_properties(DBoW2 PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${SMART_DRONE_ORB_SLAM3_LIB_DIR}"
)

set(SMART_DRONE_ORB_G2O_SOURCES
  "${SMART_DRONE_G2O_ROOT}/g2o/types/types_sba.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/types/types_six_dof_expmap.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/types/types_seven_dof_expmap.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/hyper_graph_action.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/hyper_graph.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/marginal_covariance_cholesky.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/matrix_structure.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/batch_stats.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/parameter.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/cache.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimizable_graph.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/solver.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimization_algorithm_factory.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/estimate_propagator.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/factory.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/sparse_optimizer.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/hyper_dijkstra.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/parameter_container.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimization_algorithm.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimization_algorithm_with_hessian.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimization_algorithm_levenberg.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/optimization_algorithm_gauss_newton.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/jacobian_workspace.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/robust_kernel.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/robust_kernel_factory.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/core/robust_kernel_impl.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/stuff/timeutil.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/stuff/os_specific.c"
  "${SMART_DRONE_G2O_ROOT}/g2o/stuff/string_tools.cpp"
  "${SMART_DRONE_G2O_ROOT}/g2o/stuff/property.cpp"
)

set_source_files_properties("${SMART_DRONE_G2O_ROOT}/g2o/stuff/os_specific.c" PROPERTIES LANGUAGE CXX)

add_library(g2o SHARED ${SMART_DRONE_ORB_G2O_SOURCES})
target_compile_definitions(g2o PUBLIC UNIX)
target_include_directories(g2o PUBLIC
  "${SMART_DRONE_G2O_ROOT}"
  "${SMART_DRONE_G2O_ROOT}/g2o/core"
  "${SMART_DRONE_G2O_ROOT}/g2o/types"
  "${SMART_DRONE_G2O_ROOT}/g2o/stuff"
)
target_link_libraries(g2o PUBLIC Eigen3::Eigen)
target_compile_options(g2o PRIVATE -O3 -w)
set_target_properties(g2o PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${SMART_DRONE_ORB_SLAM3_LIB_DIR}"
)

add_library(orb3_common STATIC
  "${SMART_DRONE_REPO_ROOT}/src/native/adapters/slam/orb_vocabulary.cpp"
  common/Converter.cc
  common/ImuTypes.cc
  common/Verbose.cc
  config/Config.cc
  config/Settings.cc
)
configure_orb_component(orb3_common)
target_link_libraries(orb3_common PUBLIC DBoW2 g2o Boost::serialization crypto)

add_library(orb3_camera STATIC
  camera/CameraModels/Pinhole.cpp
  camera/CameraModels/KannalaBrandt8.cpp
)
configure_orb_component(orb3_camera)
target_link_libraries(orb3_camera PUBLIC orb3_common)

add_library(orb3_features STATIC
  features/FeatureMatcherBackend.cc
  features/ORBextractor.cc
  features/ORBmatcher.cc
)
configure_orb_component(orb3_features)
target_link_libraries(orb3_features PUBLIC orb3_common orb3_camera DBoW2)
target_compile_definitions(orb3_features PRIVATE SMART_DRONE_HAS_VPI=0)
if(SMART_DRONE_ENABLE_OPENCV_CUDA_ORB)
  target_compile_definitions(orb3_features PRIVATE SMART_DRONE_HAS_OPENCV_CUDA_ORB=1)
endif()

add_library(orb3_geometry STATIC
  geometry/GeometricTools.cc
  geometry/TwoViewReconstruction.cc
)
configure_orb_component(orb3_geometry)
target_link_libraries(orb3_geometry PUBLIC orb3_common orb3_camera)

add_library(orb3_map STATIC
  "${SMART_DRONE_REPO_ROOT}/src/native/adapters/slam/visual_frame_observation_loader.cpp"
  map/MapPoint.cc
  map/Atlas.cc
  map/Map.cc
  frame/Frame.cc
  frame/FrameFactory.cc
  frame/KeyFrame.cc
)
configure_orb_component(orb3_map)
target_link_libraries(orb3_map PUBLIC orb3_common orb3_camera orb3_features orb3_geometry)

add_library(orb3_optimization STATIC
  graph_optimization/G2oTypes.cc
  graph_optimization/OptimizableTypes.cpp
  graph_optimization/OptimizationBackend.cc
  graph_optimization/Optimizer.cc
  graph_optimization/Sim3Solver.cc
  graph_optimization/MLPnPsolver.cpp
)
configure_orb_component(orb3_optimization)
target_link_libraries(orb3_optimization PUBLIC orb3_common orb3_map g2o)

add_library(orb3_place_recognition STATIC
  place_recognition/KeyFrameDatabase.cc
  place_recognition/PlaceRecognitionBackend.cc
)
configure_orb_component(orb3_place_recognition)
target_link_libraries(orb3_place_recognition PUBLIC orb3_common orb3_features orb3_map DBoW2)

add_library(orb3_runtime STATIC
  tracking/TrackedVisualDataExtractor.cc
  tracking/TrackingBackend.cc
  tracking/Tracking.cc
  loop_closing/LoopClosingBackend.cc
  mapping/LocalMapping.cc
  mapping/LocalMappingBackend.cc
  loop_closing/LoopClosing.cc
)
configure_orb_component(orb3_runtime)
target_link_libraries(orb3_runtime PUBLIC
  orb3_common
  orb3_camera
  orb3_features
  orb3_geometry
  orb3_map
  orb3_optimization
  orb3_place_recognition
)

add_library(ORB_SLAM3 SHARED system/System.cc)
target_include_directories(ORB_SLAM3 PUBLIC
  ${SMART_DRONE_ORB_MODULE_INCLUDE_DIRS}
  ${SMART_DRONE_ORB_THIRD_PARTY_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ORB_SLAM3 PUBLIC
  orb3_runtime
  Boost::serialization
  ${OpenCV_LIBS}
  crypto
)
target_compile_definitions(ORB_SLAM3 PRIVATE SMART_DRONE_HAS_VPI=0)
target_compile_options(ORB_SLAM3 PRIVATE -O3 -w)
set_target_properties(ORB_SLAM3 PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${SMART_DRONE_ORB_SLAM3_LIB_DIR}"
)
