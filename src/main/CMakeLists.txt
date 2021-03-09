
HYSLAM_ADD_SOURCES(
        System.h System.cc
        Tracking.h Tracking.cpp
        Mapping.h Mapping.cpp
        LoopClosing.h LoopClosing.cc
        ImageProcessing.h   ImageProcessing.cpp
        InterThread.h   InterThread.cpp
        ORBSLAM_datastructs.h ORBSLAM_datastructs.cc
)

set(HYSLAM_PUBLIC_INCLUDE_DIRS ${HYSLAM_PUBLIC_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
