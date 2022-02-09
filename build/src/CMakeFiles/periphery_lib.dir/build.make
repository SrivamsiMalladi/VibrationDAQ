# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build

# Utility rule file for periphery_lib.

# Include the progress variables for this target.
include src/CMakeFiles/periphery_lib.dir/progress.make

src/CMakeFiles/periphery_lib: src/CMakeFiles/periphery_lib-complete


src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-install
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-mkdir
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-patch
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-build
src/CMakeFiles/periphery_lib-complete: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/CMakeFiles
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/CMakeFiles/periphery_lib-complete
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-done

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-install: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && PREFIX=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-install

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/tmp
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E make_directory /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-mkdir

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-urlinfo.txt
src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (download, verify and extract) for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src && /usr/bin/cmake -P /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/download-periphery_lib.cmake
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src && /usr/bin/cmake -P /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/verify-periphery_lib.cmake
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src && /usr/bin/cmake -P /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/extract-periphery_lib.cmake
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-patch: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No patch step for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E echo_append
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-patch

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure: src/c-periphery-2.1.0/tmp/periphery_lib-cfgcmd.txt
src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No configure step for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E echo_append
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure

src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-build: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing build step for 'periphery_lib'"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && make
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib && /usr/bin/cmake -E touch /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-build

periphery_lib: src/CMakeFiles/periphery_lib
periphery_lib: src/CMakeFiles/periphery_lib-complete
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-install
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-mkdir
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-download
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-patch
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-configure
periphery_lib: src/c-periphery-2.1.0/src/periphery_lib-stamp/periphery_lib-build
periphery_lib: src/CMakeFiles/periphery_lib.dir/build.make

.PHONY : periphery_lib

# Rule to build all files generated by this target.
src/CMakeFiles/periphery_lib.dir/build: periphery_lib

.PHONY : src/CMakeFiles/periphery_lib.dir/build

src/CMakeFiles/periphery_lib.dir/clean:
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && $(CMAKE_COMMAND) -P CMakeFiles/periphery_lib.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/periphery_lib.dir/clean

src/CMakeFiles/periphery_lib.dir/depend:
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/CMakeFiles/periphery_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/periphery_lib.dir/depend

