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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp

# Include any dependencies generated for this target.
include CMakeFiles/cmTC_bf79a.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cmTC_bf79a.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmTC_bf79a.dir/flags.make

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o: CMakeFiles/cmTC_bf79a.dir/flags.make
CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o: /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/feature_tests.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=/home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o   -c /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/feature_tests.c

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.i: cmake_force
	@echo "Preprocessing C source to CMakeFiles/cmTC_bf79a.dir/feature_tests.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/feature_tests.c > CMakeFiles/cmTC_bf79a.dir/feature_tests.c.i

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.s: cmake_force
	@echo "Compiling C source to assembly CMakeFiles/cmTC_bf79a.dir/feature_tests.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/feature_tests.c -o CMakeFiles/cmTC_bf79a.dir/feature_tests.c.s

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.requires:

.PHONY : CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.requires

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.provides: CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.requires
	$(MAKE) -f CMakeFiles/cmTC_bf79a.dir/build.make CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.provides.build
.PHONY : CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.provides

CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.provides.build: CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o


# Object files for target cmTC_bf79a
cmTC_bf79a_OBJECTS = \
"CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o"

# External object files for target cmTC_bf79a
cmTC_bf79a_EXTERNAL_OBJECTS =

cmTC_bf79a: CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o
cmTC_bf79a: CMakeFiles/cmTC_bf79a.dir/build.make
cmTC_bf79a: CMakeFiles/cmTC_bf79a.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --progress-dir=/home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable cmTC_bf79a"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cmTC_bf79a.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmTC_bf79a.dir/build: cmTC_bf79a

.PHONY : CMakeFiles/cmTC_bf79a.dir/build

CMakeFiles/cmTC_bf79a.dir/requires: CMakeFiles/cmTC_bf79a.dir/feature_tests.c.o.requires

.PHONY : CMakeFiles/cmTC_bf79a.dir/requires

CMakeFiles/cmTC_bf79a.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cmTC_bf79a.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cmTC_bf79a.dir/clean

CMakeFiles/cmTC_bf79a.dir/depend:
	cd /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp /home/nvidia/hitsz_icra_2019/src/map_server/pi_trees/CMakeFiles/CMakeTmp/CMakeFiles/cmTC_bf79a.dir/DependInfo.cmake
.PHONY : CMakeFiles/cmTC_bf79a.dir/depend

