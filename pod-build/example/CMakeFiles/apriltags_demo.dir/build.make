# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.0.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.0.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/sahilshah/Desktop/project/software/downloads/apriltags

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build

# Include any dependencies generated for this target.
include example/CMakeFiles/apriltags_demo.dir/depend.make

# Include the progress variables for this target.
include example/CMakeFiles/apriltags_demo.dir/progress.make

# Include the compile flags for this target's objects.
include example/CMakeFiles/apriltags_demo.dir/flags.make

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o: example/CMakeFiles/apriltags_demo.dir/flags.make
example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o: ../example/apriltags_demo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o -c /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/apriltags_demo.cpp

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.i"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/apriltags_demo.cpp > CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.i

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.s"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/apriltags_demo.cpp -o CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.s

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.requires:
.PHONY : example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.requires

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.provides: example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.requires
	$(MAKE) -f example/CMakeFiles/apriltags_demo.dir/build.make example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.provides.build
.PHONY : example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.provides

example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.provides.build: example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o: example/CMakeFiles/apriltags_demo.dir/flags.make
example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o: ../example/Serial.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/apriltags_demo.dir/Serial.cpp.o -c /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/Serial.cpp

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/apriltags_demo.dir/Serial.cpp.i"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/Serial.cpp > CMakeFiles/apriltags_demo.dir/Serial.cpp.i

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/apriltags_demo.dir/Serial.cpp.s"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/sahilshah/Desktop/project/software/downloads/apriltags/example/Serial.cpp -o CMakeFiles/apriltags_demo.dir/Serial.cpp.s

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.requires:
.PHONY : example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.requires

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.provides: example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.requires
	$(MAKE) -f example/CMakeFiles/apriltags_demo.dir/build.make example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.provides.build
.PHONY : example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.provides

example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.provides.build: example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o

# Object files for target apriltags_demo
apriltags_demo_OBJECTS = \
"CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o" \
"CMakeFiles/apriltags_demo.dir/Serial.cpp.o"

# External object files for target apriltags_demo
apriltags_demo_EXTERNAL_OBJECTS =

bin/apriltags_demo: example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o
bin/apriltags_demo: example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o
bin/apriltags_demo: example/CMakeFiles/apriltags_demo.dir/build.make
bin/apriltags_demo: lib/libapriltags.a
bin/apriltags_demo: /usr/local/lib/libopencv_videostab.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_ts.a
bin/apriltags_demo: /usr/local/lib/libopencv_superres.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_stitching.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_contrib.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_nonfree.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_ocl.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_gpu.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_photo.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_objdetect.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_legacy.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_video.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_ml.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_calib3d.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_features2d.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_highgui.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_imgproc.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_flann.2.4.10.dylib
bin/apriltags_demo: /usr/local/lib/libopencv_core.2.4.10.dylib
bin/apriltags_demo: example/CMakeFiles/apriltags_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/apriltags_demo"
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/apriltags_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
example/CMakeFiles/apriltags_demo.dir/build: bin/apriltags_demo
.PHONY : example/CMakeFiles/apriltags_demo.dir/build

example/CMakeFiles/apriltags_demo.dir/requires: example/CMakeFiles/apriltags_demo.dir/apriltags_demo.cpp.o.requires
example/CMakeFiles/apriltags_demo.dir/requires: example/CMakeFiles/apriltags_demo.dir/Serial.cpp.o.requires
.PHONY : example/CMakeFiles/apriltags_demo.dir/requires

example/CMakeFiles/apriltags_demo.dir/clean:
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example && $(CMAKE_COMMAND) -P CMakeFiles/apriltags_demo.dir/cmake_clean.cmake
.PHONY : example/CMakeFiles/apriltags_demo.dir/clean

example/CMakeFiles/apriltags_demo.dir/depend:
	cd /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/sahilshah/Desktop/project/software/downloads/apriltags /Users/sahilshah/Desktop/project/software/downloads/apriltags/example /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example /Users/sahilshah/Desktop/project/software/downloads/apriltags/pod-build/example/CMakeFiles/apriltags_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : example/CMakeFiles/apriltags_demo.dir/depend

