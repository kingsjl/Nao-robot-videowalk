cmake_minimum_required(VERSION 2.8)
project(videowalk)

find_package(qibuild)

# Create a executable named videowalk
# with the source file: main.cpp
qi_create_bin(videowalk "main.cpp")


qi_use_lib(videowalk ALCOMMON ALPROXIES ALVISION OPENCV2_CORE OPENCV2_HIGHGUI)
