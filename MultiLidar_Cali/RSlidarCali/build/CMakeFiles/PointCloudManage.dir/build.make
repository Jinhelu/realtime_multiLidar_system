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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build

# Include any dependencies generated for this target.
include CMakeFiles/PointCloudManage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/PointCloudManage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/PointCloudManage.dir/flags.make

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o: CMakeFiles/PointCloudManage.dir/flags.make
CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o: ../src/PointCloudManage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o -c /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/src/PointCloudManage.cpp

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/src/PointCloudManage.cpp > CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.i

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/src/PointCloudManage.cpp -o CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.s

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.requires:

.PHONY : CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.requires

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.provides: CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.requires
	$(MAKE) -f CMakeFiles/PointCloudManage.dir/build.make CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.provides.build
.PHONY : CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.provides

CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.provides.build: CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o


# Object files for target PointCloudManage
PointCloudManage_OBJECTS = \
"CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o"

# External object files for target PointCloudManage
PointCloudManage_EXTERNAL_OBJECTS =

libPointCloudManage.so: CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o
libPointCloudManage.so: CMakeFiles/PointCloudManage.dir/build.make
libPointCloudManage.so: CMakeFiles/PointCloudManage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libPointCloudManage.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PointCloudManage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/PointCloudManage.dir/build: libPointCloudManage.so

.PHONY : CMakeFiles/PointCloudManage.dir/build

CMakeFiles/PointCloudManage.dir/requires: CMakeFiles/PointCloudManage.dir/src/PointCloudManage.cpp.o.requires

.PHONY : CMakeFiles/PointCloudManage.dir/requires

CMakeFiles/PointCloudManage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PointCloudManage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PointCloudManage.dir/clean

CMakeFiles/PointCloudManage.dir/depend:
	cd /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build /media/xiawenqiang/读书资料/曹亚宁毕业资料/代码/MultiLidar_Cali/RSlidarCali/build/CMakeFiles/PointCloudManage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PointCloudManage.dir/depend

