# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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

# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release

# Include any dependencies generated for this target.
include CMakeFiles/pyosqp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pyosqp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pyosqp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pyosqp.dir/flags.make

CMakeFiles/pyosqp.dir/src/algebra_libs.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/algebra_libs.c.o: ../src/algebra_libs.c
CMakeFiles/pyosqp.dir/src/algebra_libs.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/pyosqp.dir/src/algebra_libs.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/algebra_libs.c.o -MF CMakeFiles/pyosqp.dir/src/algebra_libs.c.o.d -o CMakeFiles/pyosqp.dir/src/algebra_libs.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/algebra_libs.c

CMakeFiles/pyosqp.dir/src/algebra_libs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/algebra_libs.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/algebra_libs.c > CMakeFiles/pyosqp.dir/src/algebra_libs.c.i

CMakeFiles/pyosqp.dir/src/algebra_libs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/algebra_libs.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/algebra_libs.c -o CMakeFiles/pyosqp.dir/src/algebra_libs.c.s

CMakeFiles/pyosqp.dir/src/auxil.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/auxil.c.o: ../src/auxil.c
CMakeFiles/pyosqp.dir/src/auxil.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/pyosqp.dir/src/auxil.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/auxil.c.o -MF CMakeFiles/pyosqp.dir/src/auxil.c.o.d -o CMakeFiles/pyosqp.dir/src/auxil.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/auxil.c

CMakeFiles/pyosqp.dir/src/auxil.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/auxil.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/auxil.c > CMakeFiles/pyosqp.dir/src/auxil.c.i

CMakeFiles/pyosqp.dir/src/auxil.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/auxil.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/auxil.c -o CMakeFiles/pyosqp.dir/src/auxil.c.s

CMakeFiles/pyosqp.dir/src/csc_math.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/csc_math.c.o: ../src/csc_math.c
CMakeFiles/pyosqp.dir/src/csc_math.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/pyosqp.dir/src/csc_math.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/csc_math.c.o -MF CMakeFiles/pyosqp.dir/src/csc_math.c.o.d -o CMakeFiles/pyosqp.dir/src/csc_math.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_math.c

CMakeFiles/pyosqp.dir/src/csc_math.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/csc_math.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_math.c > CMakeFiles/pyosqp.dir/src/csc_math.c.i

CMakeFiles/pyosqp.dir/src/csc_math.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/csc_math.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_math.c -o CMakeFiles/pyosqp.dir/src/csc_math.c.s

CMakeFiles/pyosqp.dir/src/csc_utils.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/csc_utils.c.o: ../src/csc_utils.c
CMakeFiles/pyosqp.dir/src/csc_utils.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/pyosqp.dir/src/csc_utils.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/csc_utils.c.o -MF CMakeFiles/pyosqp.dir/src/csc_utils.c.o.d -o CMakeFiles/pyosqp.dir/src/csc_utils.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_utils.c

CMakeFiles/pyosqp.dir/src/csc_utils.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/csc_utils.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_utils.c > CMakeFiles/pyosqp.dir/src/csc_utils.c.i

CMakeFiles/pyosqp.dir/src/csc_utils.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/csc_utils.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/csc_utils.c -o CMakeFiles/pyosqp.dir/src/csc_utils.c.s

CMakeFiles/pyosqp.dir/src/error.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/error.c.o: ../src/error.c
CMakeFiles/pyosqp.dir/src/error.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/pyosqp.dir/src/error.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/error.c.o -MF CMakeFiles/pyosqp.dir/src/error.c.o.d -o CMakeFiles/pyosqp.dir/src/error.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/error.c

CMakeFiles/pyosqp.dir/src/error.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/error.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/error.c > CMakeFiles/pyosqp.dir/src/error.c.i

CMakeFiles/pyosqp.dir/src/error.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/error.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/error.c -o CMakeFiles/pyosqp.dir/src/error.c.s

CMakeFiles/pyosqp.dir/src/kkt.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/kkt.c.o: ../src/kkt.c
CMakeFiles/pyosqp.dir/src/kkt.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/pyosqp.dir/src/kkt.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/kkt.c.o -MF CMakeFiles/pyosqp.dir/src/kkt.c.o.d -o CMakeFiles/pyosqp.dir/src/kkt.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/kkt.c

CMakeFiles/pyosqp.dir/src/kkt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/kkt.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/kkt.c > CMakeFiles/pyosqp.dir/src/kkt.c.i

CMakeFiles/pyosqp.dir/src/kkt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/kkt.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/kkt.c -o CMakeFiles/pyosqp.dir/src/kkt.c.s

CMakeFiles/pyosqp.dir/src/matrix.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/matrix.c.o: ../src/matrix.c
CMakeFiles/pyosqp.dir/src/matrix.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/pyosqp.dir/src/matrix.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/matrix.c.o -MF CMakeFiles/pyosqp.dir/src/matrix.c.o.d -o CMakeFiles/pyosqp.dir/src/matrix.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/matrix.c

CMakeFiles/pyosqp.dir/src/matrix.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/matrix.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/matrix.c > CMakeFiles/pyosqp.dir/src/matrix.c.i

CMakeFiles/pyosqp.dir/src/matrix.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/matrix.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/matrix.c -o CMakeFiles/pyosqp.dir/src/matrix.c.s

CMakeFiles/pyosqp.dir/src/osqp_api.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/osqp_api.c.o: ../src/osqp_api.c
CMakeFiles/pyosqp.dir/src/osqp_api.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/pyosqp.dir/src/osqp_api.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/osqp_api.c.o -MF CMakeFiles/pyosqp.dir/src/osqp_api.c.o.d -o CMakeFiles/pyosqp.dir/src/osqp_api.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/osqp_api.c

CMakeFiles/pyosqp.dir/src/osqp_api.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/osqp_api.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/osqp_api.c > CMakeFiles/pyosqp.dir/src/osqp_api.c.i

CMakeFiles/pyosqp.dir/src/osqp_api.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/osqp_api.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/osqp_api.c -o CMakeFiles/pyosqp.dir/src/osqp_api.c.s

CMakeFiles/pyosqp.dir/src/qdldl.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/qdldl.c.o: ../src/qdldl.c
CMakeFiles/pyosqp.dir/src/qdldl.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/pyosqp.dir/src/qdldl.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/qdldl.c.o -MF CMakeFiles/pyosqp.dir/src/qdldl.c.o.d -o CMakeFiles/pyosqp.dir/src/qdldl.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl.c

CMakeFiles/pyosqp.dir/src/qdldl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/qdldl.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl.c > CMakeFiles/pyosqp.dir/src/qdldl.c.i

CMakeFiles/pyosqp.dir/src/qdldl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/qdldl.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl.c -o CMakeFiles/pyosqp.dir/src/qdldl.c.s

CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o: ../src/qdldl_interface.c
CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o -MF CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o.d -o CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl_interface.c

CMakeFiles/pyosqp.dir/src/qdldl_interface.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/qdldl_interface.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl_interface.c > CMakeFiles/pyosqp.dir/src/qdldl_interface.c.i

CMakeFiles/pyosqp.dir/src/qdldl_interface.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/qdldl_interface.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/qdldl_interface.c -o CMakeFiles/pyosqp.dir/src/qdldl_interface.c.s

CMakeFiles/pyosqp.dir/src/scaling.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/scaling.c.o: ../src/scaling.c
CMakeFiles/pyosqp.dir/src/scaling.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object CMakeFiles/pyosqp.dir/src/scaling.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/scaling.c.o -MF CMakeFiles/pyosqp.dir/src/scaling.c.o.d -o CMakeFiles/pyosqp.dir/src/scaling.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/scaling.c

CMakeFiles/pyosqp.dir/src/scaling.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/scaling.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/scaling.c > CMakeFiles/pyosqp.dir/src/scaling.c.i

CMakeFiles/pyosqp.dir/src/scaling.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/scaling.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/scaling.c -o CMakeFiles/pyosqp.dir/src/scaling.c.s

CMakeFiles/pyosqp.dir/src/util.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/util.c.o: ../src/util.c
CMakeFiles/pyosqp.dir/src/util.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object CMakeFiles/pyosqp.dir/src/util.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/util.c.o -MF CMakeFiles/pyosqp.dir/src/util.c.o.d -o CMakeFiles/pyosqp.dir/src/util.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/util.c

CMakeFiles/pyosqp.dir/src/util.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/util.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/util.c > CMakeFiles/pyosqp.dir/src/util.c.i

CMakeFiles/pyosqp.dir/src/util.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/util.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/util.c -o CMakeFiles/pyosqp.dir/src/util.c.s

CMakeFiles/pyosqp.dir/src/vector.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/src/vector.c.o: ../src/vector.c
CMakeFiles/pyosqp.dir/src/vector.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object CMakeFiles/pyosqp.dir/src/vector.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/src/vector.c.o -MF CMakeFiles/pyosqp.dir/src/vector.c.o.d -o CMakeFiles/pyosqp.dir/src/vector.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/vector.c

CMakeFiles/pyosqp.dir/src/vector.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/src/vector.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/vector.c > CMakeFiles/pyosqp.dir/src/vector.c.i

CMakeFiles/pyosqp.dir/src/vector.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/src/vector.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/src/vector.c -o CMakeFiles/pyosqp.dir/src/vector.c.s

CMakeFiles/pyosqp.dir/bindings.cpp.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/bindings.cpp.o: ../bindings.cpp
CMakeFiles/pyosqp.dir/bindings.cpp.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/pyosqp.dir/bindings.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/bindings.cpp.o -MF CMakeFiles/pyosqp.dir/bindings.cpp.o.d -o CMakeFiles/pyosqp.dir/bindings.cpp.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/bindings.cpp

CMakeFiles/pyosqp.dir/bindings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pyosqp.dir/bindings.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/bindings.cpp > CMakeFiles/pyosqp.dir/bindings.cpp.i

CMakeFiles/pyosqp.dir/bindings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pyosqp.dir/bindings.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/bindings.cpp -o CMakeFiles/pyosqp.dir/bindings.cpp.s

CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o: CMakeFiles/pyosqp.dir/flags.make
CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o: ../osqp_data_workspace.c
CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o: CMakeFiles/pyosqp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building C object CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o -MF CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o.d -o CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o -c /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/osqp_data_workspace.c

CMakeFiles/pyosqp.dir/osqp_data_workspace.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/pyosqp.dir/osqp_data_workspace.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/osqp_data_workspace.c > CMakeFiles/pyosqp.dir/osqp_data_workspace.c.i

CMakeFiles/pyosqp.dir/osqp_data_workspace.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/pyosqp.dir/osqp_data_workspace.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/osqp_data_workspace.c -o CMakeFiles/pyosqp.dir/osqp_data_workspace.c.s

# Object files for target pyosqp
pyosqp_OBJECTS = \
"CMakeFiles/pyosqp.dir/src/algebra_libs.c.o" \
"CMakeFiles/pyosqp.dir/src/auxil.c.o" \
"CMakeFiles/pyosqp.dir/src/csc_math.c.o" \
"CMakeFiles/pyosqp.dir/src/csc_utils.c.o" \
"CMakeFiles/pyosqp.dir/src/error.c.o" \
"CMakeFiles/pyosqp.dir/src/kkt.c.o" \
"CMakeFiles/pyosqp.dir/src/matrix.c.o" \
"CMakeFiles/pyosqp.dir/src/osqp_api.c.o" \
"CMakeFiles/pyosqp.dir/src/qdldl.c.o" \
"CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o" \
"CMakeFiles/pyosqp.dir/src/scaling.c.o" \
"CMakeFiles/pyosqp.dir/src/util.c.o" \
"CMakeFiles/pyosqp.dir/src/vector.c.o" \
"CMakeFiles/pyosqp.dir/bindings.cpp.o" \
"CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o"

# External object files for target pyosqp
pyosqp_EXTERNAL_OBJECTS =

pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/algebra_libs.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/auxil.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/csc_math.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/csc_utils.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/error.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/kkt.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/matrix.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/osqp_api.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/qdldl.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/qdldl_interface.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/scaling.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/util.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/src/vector.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/bindings.cpp.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/osqp_data_workspace.c.o
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/build.make
pyosqp.cpython-37m-x86_64-linux-gnu.so: libosqpstatic.a
pyosqp.cpython-37m-x86_64-linux-gnu.so: CMakeFiles/pyosqp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Linking CXX shared module pyosqp.cpython-37m-x86_64-linux-gnu.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pyosqp.dir/link.txt --verbose=$(VERBOSE)
	/usr/bin/strip /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/pyosqp.cpython-37m-x86_64-linux-gnu.so

# Rule to build all files generated by this target.
CMakeFiles/pyosqp.dir/build: pyosqp.cpython-37m-x86_64-linux-gnu.so
.PHONY : CMakeFiles/pyosqp.dir/build

CMakeFiles/pyosqp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pyosqp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pyosqp.dir/clean

CMakeFiles/pyosqp.dir/depend:
	cd /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release /home/khai/SSD/Code/mcu-testing/qp_mpc_problem/generated_osqp_solver/release/CMakeFiles/pyosqp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pyosqp.dir/depend

