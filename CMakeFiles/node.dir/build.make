# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/debian/BacksubDriver_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/debian/BacksubDriver_test

# Include any dependencies generated for this target.
include CMakeFiles/node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/node.dir/flags.make

CMakeFiles/node.dir/detection/BGSDetector.cpp.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/detection/BGSDetector.cpp.o: detection/BGSDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/node.dir/detection/BGSDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/node.dir/detection/BGSDetector.cpp.o -c /home/debian/BacksubDriver_test/detection/BGSDetector.cpp

CMakeFiles/node.dir/detection/BGSDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/node.dir/detection/BGSDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/BacksubDriver_test/detection/BGSDetector.cpp > CMakeFiles/node.dir/detection/BGSDetector.cpp.i

CMakeFiles/node.dir/detection/BGSDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/node.dir/detection/BGSDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/BacksubDriver_test/detection/BGSDetector.cpp -o CMakeFiles/node.dir/detection/BGSDetector.cpp.s

CMakeFiles/node.dir/detection/BGSDetector.cpp.o.requires:

.PHONY : CMakeFiles/node.dir/detection/BGSDetector.cpp.o.requires

CMakeFiles/node.dir/detection/BGSDetector.cpp.o.provides: CMakeFiles/node.dir/detection/BGSDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/detection/BGSDetector.cpp.o.provides.build
.PHONY : CMakeFiles/node.dir/detection/BGSDetector.cpp.o.provides

CMakeFiles/node.dir/detection/BGSDetector.cpp.o.provides.build: CMakeFiles/node.dir/detection/BGSDetector.cpp.o


CMakeFiles/node.dir/detection/Detector.cpp.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/detection/Detector.cpp.o: detection/Detector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/node.dir/detection/Detector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/node.dir/detection/Detector.cpp.o -c /home/debian/BacksubDriver_test/detection/Detector.cpp

CMakeFiles/node.dir/detection/Detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/node.dir/detection/Detector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/BacksubDriver_test/detection/Detector.cpp > CMakeFiles/node.dir/detection/Detector.cpp.i

CMakeFiles/node.dir/detection/Detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/node.dir/detection/Detector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/BacksubDriver_test/detection/Detector.cpp -o CMakeFiles/node.dir/detection/Detector.cpp.s

CMakeFiles/node.dir/detection/Detector.cpp.o.requires:

.PHONY : CMakeFiles/node.dir/detection/Detector.cpp.o.requires

CMakeFiles/node.dir/detection/Detector.cpp.o.provides: CMakeFiles/node.dir/detection/Detector.cpp.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/detection/Detector.cpp.o.provides.build
.PHONY : CMakeFiles/node.dir/detection/Detector.cpp.o.provides

CMakeFiles/node.dir/detection/Detector.cpp.o.provides.build: CMakeFiles/node.dir/detection/Detector.cpp.o


CMakeFiles/node.dir/detection/NodeClient.cpp.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/detection/NodeClient.cpp.o: detection/NodeClient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/node.dir/detection/NodeClient.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/node.dir/detection/NodeClient.cpp.o -c /home/debian/BacksubDriver_test/detection/NodeClient.cpp

CMakeFiles/node.dir/detection/NodeClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/node.dir/detection/NodeClient.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/BacksubDriver_test/detection/NodeClient.cpp > CMakeFiles/node.dir/detection/NodeClient.cpp.i

CMakeFiles/node.dir/detection/NodeClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/node.dir/detection/NodeClient.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/BacksubDriver_test/detection/NodeClient.cpp -o CMakeFiles/node.dir/detection/NodeClient.cpp.s

CMakeFiles/node.dir/detection/NodeClient.cpp.o.requires:

.PHONY : CMakeFiles/node.dir/detection/NodeClient.cpp.o.requires

CMakeFiles/node.dir/detection/NodeClient.cpp.o.provides: CMakeFiles/node.dir/detection/NodeClient.cpp.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/detection/NodeClient.cpp.o.provides.build
.PHONY : CMakeFiles/node.dir/detection/NodeClient.cpp.o.provides

CMakeFiles/node.dir/detection/NodeClient.cpp.o.provides.build: CMakeFiles/node.dir/detection/NodeClient.cpp.o


CMakeFiles/node.dir/maxi.cpp.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/maxi.cpp.o: maxi.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/node.dir/maxi.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/node.dir/maxi.cpp.o -c /home/debian/BacksubDriver_test/maxi.cpp

CMakeFiles/node.dir/maxi.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/node.dir/maxi.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/BacksubDriver_test/maxi.cpp > CMakeFiles/node.dir/maxi.cpp.i

CMakeFiles/node.dir/maxi.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/node.dir/maxi.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/BacksubDriver_test/maxi.cpp -o CMakeFiles/node.dir/maxi.cpp.s

CMakeFiles/node.dir/maxi.cpp.o.requires:

.PHONY : CMakeFiles/node.dir/maxi.cpp.o.requires

CMakeFiles/node.dir/maxi.cpp.o.provides: CMakeFiles/node.dir/maxi.cpp.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/maxi.cpp.o.provides.build
.PHONY : CMakeFiles/node.dir/maxi.cpp.o.provides

CMakeFiles/node.dir/maxi.cpp.o.provides.build: CMakeFiles/node.dir/maxi.cpp.o


CMakeFiles/node.dir/drivers/xbacksub.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xbacksub.c.o: drivers/xbacksub.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/node.dir/drivers/xbacksub.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xbacksub.c.o   -c /home/debian/BacksubDriver_test/drivers/xbacksub.c

CMakeFiles/node.dir/drivers/xbacksub.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xbacksub.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xbacksub.c > CMakeFiles/node.dir/drivers/xbacksub.c.i

CMakeFiles/node.dir/drivers/xbacksub.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xbacksub.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xbacksub.c -o CMakeFiles/node.dir/drivers/xbacksub.c.s

CMakeFiles/node.dir/drivers/xbacksub.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xbacksub.c.o.requires

CMakeFiles/node.dir/drivers/xbacksub.c.o.provides: CMakeFiles/node.dir/drivers/xbacksub.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xbacksub.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xbacksub.c.o.provides

CMakeFiles/node.dir/drivers/xbacksub.c.o.provides.build: CMakeFiles/node.dir/drivers/xbacksub.c.o


CMakeFiles/node.dir/drivers/xbacksub_linux.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xbacksub_linux.c.o: drivers/xbacksub_linux.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/node.dir/drivers/xbacksub_linux.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xbacksub_linux.c.o   -c /home/debian/BacksubDriver_test/drivers/xbacksub_linux.c

CMakeFiles/node.dir/drivers/xbacksub_linux.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xbacksub_linux.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xbacksub_linux.c > CMakeFiles/node.dir/drivers/xbacksub_linux.c.i

CMakeFiles/node.dir/drivers/xbacksub_linux.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xbacksub_linux.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xbacksub_linux.c -o CMakeFiles/node.dir/drivers/xbacksub_linux.c.s

CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.requires

CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.provides: CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.provides

CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.provides.build: CMakeFiles/node.dir/drivers/xbacksub_linux.c.o


CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o: drivers/xbacksub_sinit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o   -c /home/debian/BacksubDriver_test/drivers/xbacksub_sinit.c

CMakeFiles/node.dir/drivers/xbacksub_sinit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xbacksub_sinit.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xbacksub_sinit.c > CMakeFiles/node.dir/drivers/xbacksub_sinit.c.i

CMakeFiles/node.dir/drivers/xbacksub_sinit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xbacksub_sinit.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xbacksub_sinit.c -o CMakeFiles/node.dir/drivers/xbacksub_sinit.c.s

CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.requires

CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.provides: CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.provides

CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.provides.build: CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o


CMakeFiles/node.dir/drivers/xfeature.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xfeature.c.o: drivers/xfeature.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/node.dir/drivers/xfeature.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xfeature.c.o   -c /home/debian/BacksubDriver_test/drivers/xfeature.c

CMakeFiles/node.dir/drivers/xfeature.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xfeature.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xfeature.c > CMakeFiles/node.dir/drivers/xfeature.c.i

CMakeFiles/node.dir/drivers/xfeature.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xfeature.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xfeature.c -o CMakeFiles/node.dir/drivers/xfeature.c.s

CMakeFiles/node.dir/drivers/xfeature.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xfeature.c.o.requires

CMakeFiles/node.dir/drivers/xfeature.c.o.provides: CMakeFiles/node.dir/drivers/xfeature.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xfeature.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xfeature.c.o.provides

CMakeFiles/node.dir/drivers/xfeature.c.o.provides.build: CMakeFiles/node.dir/drivers/xfeature.c.o


CMakeFiles/node.dir/drivers/xfeature_linux.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xfeature_linux.c.o: drivers/xfeature_linux.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/node.dir/drivers/xfeature_linux.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xfeature_linux.c.o   -c /home/debian/BacksubDriver_test/drivers/xfeature_linux.c

CMakeFiles/node.dir/drivers/xfeature_linux.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xfeature_linux.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xfeature_linux.c > CMakeFiles/node.dir/drivers/xfeature_linux.c.i

CMakeFiles/node.dir/drivers/xfeature_linux.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xfeature_linux.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xfeature_linux.c -o CMakeFiles/node.dir/drivers/xfeature_linux.c.s

CMakeFiles/node.dir/drivers/xfeature_linux.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xfeature_linux.c.o.requires

CMakeFiles/node.dir/drivers/xfeature_linux.c.o.provides: CMakeFiles/node.dir/drivers/xfeature_linux.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xfeature_linux.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xfeature_linux.c.o.provides

CMakeFiles/node.dir/drivers/xfeature_linux.c.o.provides.build: CMakeFiles/node.dir/drivers/xfeature_linux.c.o


CMakeFiles/node.dir/drivers/xfeature_sinit.c.o: CMakeFiles/node.dir/flags.make
CMakeFiles/node.dir/drivers/xfeature_sinit.c.o: drivers/xfeature_sinit.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CMakeFiles/node.dir/drivers/xfeature_sinit.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/node.dir/drivers/xfeature_sinit.c.o   -c /home/debian/BacksubDriver_test/drivers/xfeature_sinit.c

CMakeFiles/node.dir/drivers/xfeature_sinit.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/node.dir/drivers/xfeature_sinit.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/debian/BacksubDriver_test/drivers/xfeature_sinit.c > CMakeFiles/node.dir/drivers/xfeature_sinit.c.i

CMakeFiles/node.dir/drivers/xfeature_sinit.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/node.dir/drivers/xfeature_sinit.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/debian/BacksubDriver_test/drivers/xfeature_sinit.c -o CMakeFiles/node.dir/drivers/xfeature_sinit.c.s

CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.requires:

.PHONY : CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.requires

CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.provides: CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.requires
	$(MAKE) -f CMakeFiles/node.dir/build.make CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.provides.build
.PHONY : CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.provides

CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.provides.build: CMakeFiles/node.dir/drivers/xfeature_sinit.c.o


# Object files for target node
node_OBJECTS = \
"CMakeFiles/node.dir/detection/BGSDetector.cpp.o" \
"CMakeFiles/node.dir/detection/Detector.cpp.o" \
"CMakeFiles/node.dir/detection/NodeClient.cpp.o" \
"CMakeFiles/node.dir/maxi.cpp.o" \
"CMakeFiles/node.dir/drivers/xbacksub.c.o" \
"CMakeFiles/node.dir/drivers/xbacksub_linux.c.o" \
"CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o" \
"CMakeFiles/node.dir/drivers/xfeature.c.o" \
"CMakeFiles/node.dir/drivers/xfeature_linux.c.o" \
"CMakeFiles/node.dir/drivers/xfeature_sinit.c.o"

# External object files for target node
node_EXTERNAL_OBJECTS =

node: CMakeFiles/node.dir/detection/BGSDetector.cpp.o
node: CMakeFiles/node.dir/detection/Detector.cpp.o
node: CMakeFiles/node.dir/detection/NodeClient.cpp.o
node: CMakeFiles/node.dir/maxi.cpp.o
node: CMakeFiles/node.dir/drivers/xbacksub.c.o
node: CMakeFiles/node.dir/drivers/xbacksub_linux.c.o
node: CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o
node: CMakeFiles/node.dir/drivers/xfeature.c.o
node: CMakeFiles/node.dir/drivers/xfeature_linux.c.o
node: CMakeFiles/node.dir/drivers/xfeature_sinit.c.o
node: CMakeFiles/node.dir/build.make
node: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libboost_system.so
node: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
node: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
node: CMakeFiles/node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/debian/BacksubDriver_test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/node.dir/build: node

.PHONY : CMakeFiles/node.dir/build

CMakeFiles/node.dir/requires: CMakeFiles/node.dir/detection/BGSDetector.cpp.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/detection/Detector.cpp.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/detection/NodeClient.cpp.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/maxi.cpp.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xbacksub.c.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xbacksub_linux.c.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xbacksub_sinit.c.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xfeature.c.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xfeature_linux.c.o.requires
CMakeFiles/node.dir/requires: CMakeFiles/node.dir/drivers/xfeature_sinit.c.o.requires

.PHONY : CMakeFiles/node.dir/requires

CMakeFiles/node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/node.dir/clean

CMakeFiles/node.dir/depend:
	cd /home/debian/BacksubDriver_test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/debian/BacksubDriver_test /home/debian/BacksubDriver_test /home/debian/BacksubDriver_test /home/debian/BacksubDriver_test /home/debian/BacksubDriver_test/CMakeFiles/node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/node.dir/depend

