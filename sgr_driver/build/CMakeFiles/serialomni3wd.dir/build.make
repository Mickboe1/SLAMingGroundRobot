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
CMAKE_SOURCE_DIR = /home/foitn/catkin_ws/src/serialomni3wd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/foitn/catkin_ws/src/serialomni3wd/build

# Include any dependencies generated for this target.
include CMakeFiles/serialomni3wd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/serialomni3wd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serialomni3wd.dir/flags.make

CMakeFiles/serialomni3wd.dir/src/serial.cc.o: CMakeFiles/serialomni3wd.dir/flags.make
CMakeFiles/serialomni3wd.dir/src/serial.cc.o: ../src/serial.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/foitn/catkin_ws/src/serialomni3wd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serialomni3wd.dir/src/serial.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialomni3wd.dir/src/serial.cc.o -c /home/foitn/catkin_ws/src/serialomni3wd/src/serial.cc

CMakeFiles/serialomni3wd.dir/src/serial.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialomni3wd.dir/src/serial.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/foitn/catkin_ws/src/serialomni3wd/src/serial.cc > CMakeFiles/serialomni3wd.dir/src/serial.cc.i

CMakeFiles/serialomni3wd.dir/src/serial.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialomni3wd.dir/src/serial.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/foitn/catkin_ws/src/serialomni3wd/src/serial.cc -o CMakeFiles/serialomni3wd.dir/src/serial.cc.s

CMakeFiles/serialomni3wd.dir/src/serial.cc.o.requires:

.PHONY : CMakeFiles/serialomni3wd.dir/src/serial.cc.o.requires

CMakeFiles/serialomni3wd.dir/src/serial.cc.o.provides: CMakeFiles/serialomni3wd.dir/src/serial.cc.o.requires
	$(MAKE) -f CMakeFiles/serialomni3wd.dir/build.make CMakeFiles/serialomni3wd.dir/src/serial.cc.o.provides.build
.PHONY : CMakeFiles/serialomni3wd.dir/src/serial.cc.o.provides

CMakeFiles/serialomni3wd.dir/src/serial.cc.o.provides.build: CMakeFiles/serialomni3wd.dir/src/serial.cc.o


CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o: CMakeFiles/serialomni3wd.dir/flags.make
CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o: ../src/impl/unix.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/foitn/catkin_ws/src/serialomni3wd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o -c /home/foitn/catkin_ws/src/serialomni3wd/src/impl/unix.cc

CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/foitn/catkin_ws/src/serialomni3wd/src/impl/unix.cc > CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.i

CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/foitn/catkin_ws/src/serialomni3wd/src/impl/unix.cc -o CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.s

CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.requires:

.PHONY : CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.requires

CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.provides: CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.requires
	$(MAKE) -f CMakeFiles/serialomni3wd.dir/build.make CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.provides.build
.PHONY : CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.provides

CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.provides.build: CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o


CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o: CMakeFiles/serialomni3wd.dir/flags.make
CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o: ../src/impl/list_ports/list_ports_linux.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/foitn/catkin_ws/src/serialomni3wd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o -c /home/foitn/catkin_ws/src/serialomni3wd/src/impl/list_ports/list_ports_linux.cc

CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/foitn/catkin_ws/src/serialomni3wd/src/impl/list_ports/list_ports_linux.cc > CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.i

CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/foitn/catkin_ws/src/serialomni3wd/src/impl/list_ports/list_ports_linux.cc -o CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.s

CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.requires:

.PHONY : CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.requires

CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.provides: CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.requires
	$(MAKE) -f CMakeFiles/serialomni3wd.dir/build.make CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.provides.build
.PHONY : CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.provides

CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.provides.build: CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o


# Object files for target serialomni3wd
serialomni3wd_OBJECTS = \
"CMakeFiles/serialomni3wd.dir/src/serial.cc.o" \
"CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o" \
"CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o"

# External object files for target serialomni3wd
serialomni3wd_EXTERNAL_OBJECTS =

devel/lib/libserialomni3wd.so: CMakeFiles/serialomni3wd.dir/src/serial.cc.o
devel/lib/libserialomni3wd.so: CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o
devel/lib/libserialomni3wd.so: CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o
devel/lib/libserialomni3wd.so: CMakeFiles/serialomni3wd.dir/build.make
devel/lib/libserialomni3wd.so: CMakeFiles/serialomni3wd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/foitn/catkin_ws/src/serialomni3wd/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library devel/lib/libserialomni3wd.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialomni3wd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serialomni3wd.dir/build: devel/lib/libserialomni3wd.so

.PHONY : CMakeFiles/serialomni3wd.dir/build

CMakeFiles/serialomni3wd.dir/requires: CMakeFiles/serialomni3wd.dir/src/serial.cc.o.requires
CMakeFiles/serialomni3wd.dir/requires: CMakeFiles/serialomni3wd.dir/src/impl/unix.cc.o.requires
CMakeFiles/serialomni3wd.dir/requires: CMakeFiles/serialomni3wd.dir/src/impl/list_ports/list_ports_linux.cc.o.requires

.PHONY : CMakeFiles/serialomni3wd.dir/requires

CMakeFiles/serialomni3wd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serialomni3wd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serialomni3wd.dir/clean

CMakeFiles/serialomni3wd.dir/depend:
	cd /home/foitn/catkin_ws/src/serialomni3wd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/foitn/catkin_ws/src/serialomni3wd /home/foitn/catkin_ws/src/serialomni3wd /home/foitn/catkin_ws/src/serialomni3wd/build /home/foitn/catkin_ws/src/serialomni3wd/build /home/foitn/catkin_ws/src/serialomni3wd/build/CMakeFiles/serialomni3wd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serialomni3wd.dir/depend
