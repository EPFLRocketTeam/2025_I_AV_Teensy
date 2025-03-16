# cmake_minimum_required(VERSION 3.27)
# project(controller_proto)

# set(CMAKE_CXX_STANDARD 17)


# Necessary to use matplotlibcpp
# find_package(Python3 COMPONENTS Development NumPy)

set(HEADERS
    inc/Controller.h
    inc/PID.h
    inc/DroneController.h
    inc/Vec3.h

    # test/inc/csv.h
    # test/inc/matplotlibcpp.h
    # test/inc/catch.hpp
)
set(SOURCES
    src/PID.cpp
    src/Controller.cpp

    # test/ControllerTest.cpp
    # test/ControllerTest.cpp
)

# add_executable(controller_proto ${SOURCES} ${HEADERS})
# target_link_libraries(controller_proto PRIVATE Python3::Python Python3::NumPy)

# Make control available as a library
add_library(control ${SOURCES} ${HEADERS})
target_include_directories(control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
