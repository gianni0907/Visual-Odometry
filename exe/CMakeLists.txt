link_libraries(${OpenCV_LIBS})

# #ds cam_test: projection test
# add_executable(cam_test cam_test.cpp
#     ../src/points_utils.cpp
#     ../src/camera.cpp
# )
# target_link_libraries(cam_test opencv_highgui)

# #ds epipolar_test: test triangulation,fundamental estimation, transform estimation
# add_executable(epipolar_test epipolar_test.cpp
#     ../src/points_utils.cpp
#     ../src/camera.cpp
#     ../src/epipolar.cpp
# )
# target_link_libraries(epipolar_test opencv_highgui)

# #ds picp_test: test projective_icp solver
# add_executable(picp_test picp_test.cpp
#     ../src/points_utils.cpp
#     ../src/camera.cpp
#     ../src/epipolar.cpp
#     ../src/projective_icp.cpp
# )
# target_link_libraries(picp_test opencv_highgui)

# #ds complete_test: test triangulation for initialization and then picp
# add_executable(complete_test complete_test.cpp
#     ../src/points_utils.cpp
#     ../src/camera.cpp
#     ../src/epipolar.cpp
#     ../src/projective_icp.cpp
# )
# target_link_libraries(complete_test opencv_highgui)

#ds read_test: test read functions
add_executable(read_test read_test.cpp
    ../src/points_utils.cpp
    ../src/camera.cpp
    ../src/read_data.cpp
)
target_link_libraries(read_test opencv_highgui)