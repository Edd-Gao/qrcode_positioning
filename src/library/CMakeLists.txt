cmake_minimum_required (VERSION 2.8.3)

file(GLOB libraryHeaders *.h *.hpp)
file(GLOB librarySource *.cpp *.c)
find_package(OpenCV REQUIRED)
find_library(libzbar.so REQUIRED)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_library(QRCodeStateEstimation ${librarySource} ${libraryHeaders})
target_link_libraries(QRCodeStateEstimation opencv_core opencv_highgui opencv_calib3d opencv_imgproc libzbar.so)

