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
CMAKE_SOURCE_DIR = /home/jixingwu/Thiredpart/slam_deepBule/PA3/6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/draw_traj.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/draw_traj.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/draw_traj.dir/flags.make

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o: CMakeFiles/draw_traj.dir/flags.make
CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o: ../src/draw_trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o -c /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/src/draw_trajectory.cpp

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/src/draw_trajectory.cpp > CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.i

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/src/draw_trajectory.cpp -o CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.s

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.requires:

.PHONY : CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.requires

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.provides: CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.requires
	$(MAKE) -f CMakeFiles/draw_traj.dir/build.make CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.provides.build
.PHONY : CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.provides

CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.provides.build: CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o


# Object files for target draw_traj
draw_traj_OBJECTS = \
"CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o"

# External object files for target draw_traj
draw_traj_EXTERNAL_OBJECTS =

libdraw_traj.so: CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o
libdraw_traj.so: CMakeFiles/draw_traj.dir/build.make
libdraw_traj.so: /usr/local/lib/libpangolin.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libGL.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libGLEW.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libwayland-client.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libwayland-egl.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libwayland-cursor.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libSM.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libICE.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libX11.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libXext.so
libdraw_traj.so: /usr/lib/libOpenNI.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libpng.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libz.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/libtiff.so
libdraw_traj.so: /usr/lib/x86_64-linux-gnu/liblz4.so
libdraw_traj.so: CMakeFiles/draw_traj.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libdraw_traj.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/draw_traj.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/draw_traj.dir/build: libdraw_traj.so

.PHONY : CMakeFiles/draw_traj.dir/build

CMakeFiles/draw_traj.dir/requires: CMakeFiles/draw_traj.dir/src/draw_trajectory.cpp.o.requires

.PHONY : CMakeFiles/draw_traj.dir/requires

CMakeFiles/draw_traj.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/draw_traj.dir/cmake_clean.cmake
.PHONY : CMakeFiles/draw_traj.dir/clean

CMakeFiles/draw_traj.dir/depend:
	cd /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jixingwu/Thiredpart/slam_deepBule/PA3/6 /home/jixingwu/Thiredpart/slam_deepBule/PA3/6 /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug /home/jixingwu/Thiredpart/slam_deepBule/PA3/6/cmake-build-debug/CMakeFiles/draw_traj.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/draw_traj.dir/depend

