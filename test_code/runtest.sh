#!/bin/sh

# Script to compile targeted source codes and run the compiled binary.
# The command supports upto 3 source files and compile flags as arguments.
#
# $./runtest.sh source1.cpp source2.cpp
#
# Add -DDEBUG like below to enable debug messages
# $./runtest.sh source1.cpp source2.cpp -DDEBUG
#
# Example:
# 
# Specifying 1 argument:
# $./runtest.sh Q3_35_Get_Pseudo_Ranges_main.cpp
#
# Specifying 2 arguments:
# $./runtest.sh Q3_37_Coding_the_Observation_Model_main.cpp multiv_gauss.cpp
# 
# Specifying debug option
# $ ./runtest.sh Q3_39_Coding_the_Full_Filter_main.cpp -DDEBUG

g++ $1 $2 $3 -std=c++11 && ./a.out