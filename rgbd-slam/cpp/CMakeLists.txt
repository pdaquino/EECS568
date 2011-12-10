cmake_minimum_required (VERSION 2.6)
project (RgbdSlam)
FIND_PACKAGE( OpenCV REQUIRED )
find_package(JNI REQUIRED)

if (JNI_FOUND)
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
    message (STATUS "JNI_LIBRARIES=${JNI_LIBRARIES}")
    message (STATUS "JNI_LIBRARIES_DIR=${JNI_LIBRARIES_DIRS}")
endif()

file(GLOB RgbdSlam_SOURCES "*.cpp" "*.c")
file(GLOB RgbdSlam_INCLUDES ".hpp" "*.h")
#add_definitions(-std=c++0x)
include_directories(${JNI_INCLUDE_DIRS})

add_executable(FeatureDetector ${RgbdSlam_SOURCES} ${RgbdSlam_INCLUDES} )
target_link_libraries(FeatureDetector ${OpenCV_LIBS})

add_library(opencvwrapper SHARED ${RgbdSlam_SOURCES} ${RgbdSlam_INCLUDES})
target_link_libraries(opencvwrapper ${OpenCV_LIBS})
