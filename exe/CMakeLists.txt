link_libraries(${OpenCV_LIBS})

#ds cam_test: projection test
add_executable(cam_test cam_test.cpp
    ../src/points_utils.cpp
    ../src/camera.cpp
)
target_link_libraries(cam_test opencv_highgui)