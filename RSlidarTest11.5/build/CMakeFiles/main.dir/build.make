# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rbt/cyn/RSlidarTest11.5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rbt/cyn/RSlidarTest11.5/build

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rbt/cyn/RSlidarTest11.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/main.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/main.cpp.o -c /home/rbt/cyn/RSlidarTest11.5/src/main.cpp

CMakeFiles/main.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rbt/cyn/RSlidarTest11.5/src/main.cpp > CMakeFiles/main.dir/src/main.cpp.i

CMakeFiles/main.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rbt/cyn/RSlidarTest11.5/src/main.cpp -o CMakeFiles/main.dir/src/main.cpp.s

CMakeFiles/main.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/main.dir/src/main.cpp.o.requires

CMakeFiles/main.dir/src/main.cpp.o.provides: CMakeFiles/main.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/main.dir/src/main.cpp.o.provides

CMakeFiles/main.dir/src/main.cpp.o.provides.build: CMakeFiles/main.dir/src/main.cpp.o


# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/main.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS =

../bin/main: CMakeFiles/main.dir/src/main.cpp.o
../bin/main: CMakeFiles/main.dir/build.make
../bin/main: libLidarMap.so
../bin/main: libPointCloudManage.so
../bin/main: libgrid_manage.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/main: /usr/lib/libpcl_common.so
../bin/main: /usr/lib/libpcl_octree.so
../bin/main: /usr/lib/libpcl_io.so
../bin/main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/main: /usr/lib/libpcl_kdtree.so
../bin/main: /usr/lib/libpcl_search.so
../bin/main: /usr/lib/libpcl_sample_consensus.so
../bin/main: /usr/lib/libpcl_filters.so
../bin/main: /usr/lib/libpcl_features.so
../bin/main: /usr/lib/libpcl_segmentation.so
../bin/main: /usr/lib/libpcl_visualization.so
../bin/main: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/main: /usr/lib/libpcl_surface.so
../bin/main: /usr/lib/libpcl_registration.so
../bin/main: /usr/lib/libpcl_keypoints.so
../bin/main: /usr/lib/libpcl_tracking.so
../bin/main: /usr/lib/libpcl_recognition.so
../bin/main: /usr/lib/libpcl_outofcore.so
../bin/main: /usr/lib/libpcl_people.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/main: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/main: /usr/lib/x86_64-linux-gnu/libqhull.so
../bin/main: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/main: /usr/local/lib/libopencv_dnn.so.3.4.1
../bin/main: /usr/local/lib/libopencv_ml.so.3.4.1
../bin/main: /usr/local/lib/libopencv_objdetect.so.3.4.1
../bin/main: /usr/local/lib/libopencv_shape.so.3.4.1
../bin/main: /usr/local/lib/libopencv_stitching.so.3.4.1
../bin/main: /usr/local/lib/libopencv_superres.so.3.4.1
../bin/main: /usr/local/lib/libopencv_videostab.so.3.4.1
../bin/main: /usr/local/lib/libopencv_viz.so.3.4.1
../bin/main: /usr/lib/libpcl_common.so
../bin/main: /usr/lib/libpcl_octree.so
../bin/main: /usr/lib/libpcl_io.so
../bin/main: /usr/lib/libpcl_kdtree.so
../bin/main: /usr/lib/libpcl_search.so
../bin/main: /usr/lib/libpcl_sample_consensus.so
../bin/main: /usr/lib/libpcl_filters.so
../bin/main: /usr/lib/libpcl_features.so
../bin/main: /usr/lib/libpcl_segmentation.so
../bin/main: /usr/lib/libpcl_visualization.so
../bin/main: /usr/lib/libpcl_surface.so
../bin/main: /usr/lib/libpcl_registration.so
../bin/main: /usr/lib/libpcl_keypoints.so
../bin/main: /usr/lib/libpcl_tracking.so
../bin/main: /usr/lib/libpcl_recognition.so
../bin/main: /usr/lib/libpcl_outofcore.so
../bin/main: /usr/lib/libpcl_people.so
../bin/main: /usr/local/lib/libopencv_calib3d.so.3.4.1
../bin/main: /usr/local/lib/libopencv_features2d.so.3.4.1
../bin/main: /usr/local/lib/libopencv_flann.so.3.4.1
../bin/main: /usr/local/lib/libopencv_highgui.so.3.4.1
../bin/main: /usr/local/lib/libopencv_photo.so.3.4.1
../bin/main: /usr/local/lib/libopencv_video.so.3.4.1
../bin/main: /usr/local/lib/libopencv_videoio.so.3.4.1
../bin/main: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
../bin/main: /usr/local/lib/libopencv_imgproc.so.3.4.1
../bin/main: /usr/lib/libvtkGenericFiltering.so.5.10.1
../bin/main: /usr/lib/libvtkGeovis.so.5.10.1
../bin/main: /usr/lib/libvtkCharts.so.5.10.1
../bin/main: /usr/lib/libvtkViews.so.5.10.1
../bin/main: /usr/lib/libvtkInfovis.so.5.10.1
../bin/main: /usr/lib/libvtkWidgets.so.5.10.1
../bin/main: /usr/lib/libvtkVolumeRendering.so.5.10.1
../bin/main: /usr/lib/libvtkHybrid.so.5.10.1
../bin/main: /usr/lib/libvtkParallel.so.5.10.1
../bin/main: /usr/lib/libvtkRendering.so.5.10.1
../bin/main: /usr/lib/libvtkImaging.so.5.10.1
../bin/main: /usr/lib/libvtkGraphics.so.5.10.1
../bin/main: /usr/lib/libvtkIO.so.5.10.1
../bin/main: /usr/lib/libvtkFiltering.so.5.10.1
../bin/main: /usr/lib/libvtkCommon.so.5.10.1
../bin/main: /usr/lib/libvtksys.so.5.10.1
../bin/main: /usr/local/lib/libopencv_core.so.3.4.1
../bin/main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rbt/cyn/RSlidarTest11.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: ../bin/main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/requires: CMakeFiles/main.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/main.dir/requires

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend:
	cd /home/rbt/cyn/RSlidarTest11.5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rbt/cyn/RSlidarTest11.5 /home/rbt/cyn/RSlidarTest11.5 /home/rbt/cyn/RSlidarTest11.5/build /home/rbt/cyn/RSlidarTest11.5/build /home/rbt/cyn/RSlidarTest11.5/build/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

