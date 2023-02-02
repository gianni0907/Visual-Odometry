link_libraries(${OpenCV_LIBS})

#ds cam_test: projection test
add_executable(cam_test cam_test.cpp
    ../src/points_utils.cpp
    ../src/camera.cpp
    ../src/epipolar.cpp
)
target_link_libraries(cam_test opencv_highgui)

#ds epipolar_test: test triangulation and essential extraction
add_executable(epipolar_test epipolar_test.cpp
    ../src/points_utils.cpp
    ../src/camera.cpp
    ../src/epipolar.cpp
)
target_link_libraries(epipolar_test opencv_highgui)