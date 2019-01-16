#include <iostream>
#include "help_functions.h"

float value = 5.5; // TODO: assign a value, the difference in distances
float parameter = 5.0; // set as control parameter or observation measurement
float stdev = 1.0; // position or observation standard deviation

int main() {

  float prob = Helpers::normpdf(value, parameter, stdev);

  std::cout << prob << std::endl;

  return 0;
}