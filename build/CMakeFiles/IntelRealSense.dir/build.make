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
CMAKE_SOURCE_DIR = /home/nano/Monika/IntelRealSense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nano/Monika/IntelRealSense/build

# Include any dependencies generated for this target.
include CMakeFiles/IntelRealSense.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/IntelRealSense.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/IntelRealSense.dir/flags.make

CMakeFiles/IntelRealSense.dir/src/main.cpp.o: CMakeFiles/IntelRealSense.dir/flags.make
CMakeFiles/IntelRealSense.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nano/Monika/IntelRealSense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/IntelRealSense.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/IntelRealSense.dir/src/main.cpp.o -c /home/nano/Monika/IntelRealSense/src/main.cpp

CMakeFiles/IntelRealSense.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IntelRealSense.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nano/Monika/IntelRealSense/src/main.cpp > CMakeFiles/IntelRealSense.dir/src/main.cpp.i

CMakeFiles/IntelRealSense.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IntelRealSense.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nano/Monika/IntelRealSense/src/main.cpp -o CMakeFiles/IntelRealSense.dir/src/main.cpp.s

CMakeFiles/IntelRealSense.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/IntelRealSense.dir/src/main.cpp.o.requires

CMakeFiles/IntelRealSense.dir/src/main.cpp.o.provides: CMakeFiles/IntelRealSense.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/IntelRealSense.dir/build.make CMakeFiles/IntelRealSense.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/IntelRealSense.dir/src/main.cpp.o.provides

CMakeFiles/IntelRealSense.dir/src/main.cpp.o.provides.build: CMakeFiles/IntelRealSense.dir/src/main.cpp.o


CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o: CMakeFiles/IntelRealSense.dir/flags.make
CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o: ../src/AHRS/FusionAhrs.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nano/Monika/IntelRealSense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o   -c /home/nano/Monika/IntelRealSense/src/AHRS/FusionAhrs.c

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nano/Monika/IntelRealSense/src/AHRS/FusionAhrs.c > CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.i

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nano/Monika/IntelRealSense/src/AHRS/FusionAhrs.c -o CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.s

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.requires:

.PHONY : CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.requires

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.provides: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.requires
	$(MAKE) -f CMakeFiles/IntelRealSense.dir/build.make CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.provides.build
.PHONY : CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.provides

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.provides.build: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o


CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o: CMakeFiles/IntelRealSense.dir/flags.make
CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o: ../src/AHRS/FusionBias.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nano/Monika/IntelRealSense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o   -c /home/nano/Monika/IntelRealSense/src/AHRS/FusionBias.c

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nano/Monika/IntelRealSense/src/AHRS/FusionBias.c > CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.i

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nano/Monika/IntelRealSense/src/AHRS/FusionBias.c -o CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.s

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.requires:

.PHONY : CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.requires

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.provides: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.requires
	$(MAKE) -f CMakeFiles/IntelRealSense.dir/build.make CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.provides.build
.PHONY : CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.provides

CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.provides.build: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o


# Object files for target IntelRealSense
IntelRealSense_OBJECTS = \
"CMakeFiles/IntelRealSense.dir/src/main.cpp.o" \
"CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o" \
"CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o"

# External object files for target IntelRealSense
IntelRealSense_EXTERNAL_OBJECTS =

IntelRealSense: CMakeFiles/IntelRealSense.dir/src/main.cpp.o
IntelRealSense: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o
IntelRealSense: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o
IntelRealSense: CMakeFiles/IntelRealSense.dir/build.make
IntelRealSense: /usr/lib/aarch64-linux-gnu/librealsense2.so.2.48.0
IntelRealSense: CMakeFiles/IntelRealSense.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nano/Monika/IntelRealSense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable IntelRealSense"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IntelRealSense.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/IntelRealSense.dir/build: IntelRealSense

.PHONY : CMakeFiles/IntelRealSense.dir/build

CMakeFiles/IntelRealSense.dir/requires: CMakeFiles/IntelRealSense.dir/src/main.cpp.o.requires
CMakeFiles/IntelRealSense.dir/requires: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionAhrs.c.o.requires
CMakeFiles/IntelRealSense.dir/requires: CMakeFiles/IntelRealSense.dir/src/AHRS/FusionBias.c.o.requires

.PHONY : CMakeFiles/IntelRealSense.dir/requires

CMakeFiles/IntelRealSense.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/IntelRealSense.dir/cmake_clean.cmake
.PHONY : CMakeFiles/IntelRealSense.dir/clean

CMakeFiles/IntelRealSense.dir/depend:
	cd /home/nano/Monika/IntelRealSense/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nano/Monika/IntelRealSense /home/nano/Monika/IntelRealSense /home/nano/Monika/IntelRealSense/build /home/nano/Monika/IntelRealSense/build /home/nano/Monika/IntelRealSense/build/CMakeFiles/IntelRealSense.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/IntelRealSense.dir/depend

