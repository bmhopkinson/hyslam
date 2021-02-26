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
set(CMAKE_SKIP_BUILD_RPATH FALSE)  #otherwise rpath is not properly set to run from build directory - expects install to default location

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
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.a
${CUDA_LIBRARIES}
)

#install
set(INSTALL_LIBDIR "/usr/local/lib")
set(INSTALL_INCLUDEDIR "/usr/local/include")
install(TARGETS ${PROJECT_NAME} 
	LIBRARY DESTINATION ${INSTALL_LIBDIR}
)

install(DIRECTORY .
		DESTINATION "${INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
		FILES_MATCHING PATTERN "*.h"
)

