# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /opt/CLion-2019.3.3/clion-2019.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/CLion-2019.3.3/clion-2019.3.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wbzhang/catkin_optflow/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wbzhang/catkin_optflow/build

# Include any dependencies generated for this target.
include optflow_obs/CMakeFiles/optflowLib.dir/depend.make

# Include the progress variables for this target.
include optflow_obs/CMakeFiles/optflowLib.dir/progress.make

# Include the compile flags for this target's objects.
include optflow_obs/CMakeFiles/optflowLib.dir/flags.make

optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o: optflow_obs/CMakeFiles/optflowLib.dir/flags.make
optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o: /home/wbzhang/catkin_optflow/src/optflow_obs/src/opflowAvoidanceClass.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wbzhang/catkin_optflow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o"
	cd /home/wbzhang/catkin_optflow/build/optflow_obs && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o -c /home/wbzhang/catkin_optflow/src/optflow_obs/src/opflowAvoidanceClass.cpp

optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.i"
	cd /home/wbzhang/catkin_optflow/build/optflow_obs && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wbzhang/catkin_optflow/src/optflow_obs/src/opflowAvoidanceClass.cpp > CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.i

optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.s"
	cd /home/wbzhang/catkin_optflow/build/optflow_obs && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wbzhang/catkin_optflow/src/optflow_obs/src/opflowAvoidanceClass.cpp -o CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.s

# Object files for target optflowLib
optflowLib_OBJECTS = \
"CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o"

# External object files for target optflowLib
optflowLib_EXTERNAL_OBJECTS =

/home/wbzhang/catkin_optflow/devel/lib/liboptflowLib.so: optflow_obs/CMakeFiles/optflowLib.dir/src/opflowAvoidanceClass.cpp.o
/home/wbzhang/catkin_optflow/devel/lib/liboptflowLib.so: optflow_obs/CMakeFiles/optflowLib.dir/build.make
/home/wbzhang/catkin_optflow/devel/lib/liboptflowLib.so: optflow_obs/CMakeFiles/optflowLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wbzhang/catkin_optflow/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/wbzhang/catkin_optflow/devel/lib/liboptflowLib.so"
	cd /home/wbzhang/catkin_optflow/build/optflow_obs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optflowLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optflow_obs/CMakeFiles/optflowLib.dir/build: /home/wbzhang/catkin_optflow/devel/lib/liboptflowLib.so

.PHONY : optflow_obs/CMakeFiles/optflowLib.dir/build

optflow_obs/CMakeFiles/optflowLib.dir/clean:
	cd /home/wbzhang/catkin_optflow/build/optflow_obs && $(CMAKE_COMMAND) -P CMakeFiles/optflowLib.dir/cmake_clean.cmake
.PHONY : optflow_obs/CMakeFiles/optflowLib.dir/clean

optflow_obs/CMakeFiles/optflowLib.dir/depend:
	cd /home/wbzhang/catkin_optflow/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wbzhang/catkin_optflow/src /home/wbzhang/catkin_optflow/src/optflow_obs /home/wbzhang/catkin_optflow/build /home/wbzhang/catkin_optflow/build/optflow_obs /home/wbzhang/catkin_optflow/build/optflow_obs/CMakeFiles/optflowLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : optflow_obs/CMakeFiles/optflowLib.dir/depend
