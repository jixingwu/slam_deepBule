# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/jixingwu/slam_deepBule/PA5/3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/GNBA_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/GNBA_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/GNBA_node.dir/flags.make

CMakeFiles/GNBA_node.dir/GN-BA.cpp.o: CMakeFiles/GNBA_node.dir/flags.make
CMakeFiles/GNBA_node.dir/GN-BA.cpp.o: ../GN-BA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/GNBA_node.dir/GN-BA.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GNBA_node.dir/GN-BA.cpp.o -c /home/jixingwu/slam_deepBule/PA5/3/GN-BA.cpp

CMakeFiles/GNBA_node.dir/GN-BA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GNBA_node.dir/GN-BA.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jixingwu/slam_deepBule/PA5/3/GN-BA.cpp > CMakeFiles/GNBA_node.dir/GN-BA.cpp.i

CMakeFiles/GNBA_node.dir/GN-BA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GNBA_node.dir/GN-BA.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jixingwu/slam_deepBule/PA5/3/GN-BA.cpp -o CMakeFiles/GNBA_node.dir/GN-BA.cpp.s

CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.requires:

.PHONY : CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.requires

CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.provides: CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.requires
	$(MAKE) -f CMakeFiles/GNBA_node.dir/build.make CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.provides.build
.PHONY : CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.provides

CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.provides.build: CMakeFiles/GNBA_node.dir/GN-BA.cpp.o


# Object files for target GNBA_node
GNBA_node_OBJECTS = \
"CMakeFiles/GNBA_node.dir/GN-BA.cpp.o"

# External object files for target GNBA_node
GNBA_node_EXTERNAL_OBJECTS =

GNBA_node: CMakeFiles/GNBA_node.dir/GN-BA.cpp.o
GNBA_node: CMakeFiles/GNBA_node.dir/build.make
GNBA_node: /home/jixingwu/Thiredpart/Sophus/build/libSophus.so
GNBA_node: CMakeFiles/GNBA_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable GNBA_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GNBA_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/GNBA_node.dir/build: GNBA_node

.PHONY : CMakeFiles/GNBA_node.dir/build

CMakeFiles/GNBA_node.dir/requires: CMakeFiles/GNBA_node.dir/GN-BA.cpp.o.requires

.PHONY : CMakeFiles/GNBA_node.dir/requires

CMakeFiles/GNBA_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/GNBA_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/GNBA_node.dir/clean

CMakeFiles/GNBA_node.dir/depend:
	cd /home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jixingwu/slam_deepBule/PA5/3 /home/jixingwu/slam_deepBule/PA5/3 /home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug /home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug /home/jixingwu/slam_deepBule/PA5/3/cmake-build-debug/CMakeFiles/GNBA_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/GNBA_node.dir/depend

