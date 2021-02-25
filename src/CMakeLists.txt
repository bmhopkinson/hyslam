add_subdirectory(core)
add_subdirectory(estimators)
add_subdirectory(features)
add_subdirectory(initializers)
add_subdirectory(optimizers)
add_subdirectory(slam)
add_subdirectory(util)
add_subdirectory(viz)


#message(STATUS "HYSLAM_SOURCES" ${HYSLAM_SOURCES})
#message(STATUS "HYSLAM_INCLUDES" ${HYSLAM_PUBLIC_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  ${HYSLAM_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${HYSLAM_PUBLIC_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
#dataproc
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${G2O_LIBS}
${G2O_TYPES_SLAM3D}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${CUDA_LIBRARIES}
)

