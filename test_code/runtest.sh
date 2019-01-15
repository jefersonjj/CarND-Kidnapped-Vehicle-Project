#!/bin/sh

# Script to compile targeted source codes and run the compiled binary.
# The command supports upto 2 source files as arguments.
#
# $./runtest.sh source1.cpp source2.cpp
#
# Example:
# 
# Specifying 1 argument:
# $./runtest.sh Q3_35_Get_Pseudo_Ranges_main.cpp
#
# Specifying 2 arguments:
# $./runtest.sh Q3_37_Coding_the_Observation_Model_main.cpp multiv_gauss.cpp

g++ $1 $2 -std=c++11 && ./a.out