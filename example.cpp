/* Compile and run me as follows:
 *   g++ example.cpp -I/usr/include/python2.7 -lpython2.7
 *   ./a.out
 *
 * It may be required to use an additional flag to compile (but I think only on python3+):
 *   g++ example.cpp -DWITHOUT_NUMPY -I/usr/include/python2.7 -lpython2.7
 *   ./a.out
 *
 * My output will be written to:
 *   plot.pdf
 *
 * matplotlib.h is a wrapper for python's matplotlib. The code was taken from here:
 *   https://github.com/lava/matplotlib-cpp/blob/master/matplotlibcpp.h
 * and modified to comment out references to 'xkcd'. This was done because OSC has matplotlib installed for python 2.7 only, but the c++ wrapper expects python3.6+. It would not run without commenting these out. It is difficult to install matplotlib for python3.6 on OSC. 
 *  -SO
 */

#include <iostream>
#include <vector>
#include "matplotlib.h"

namespace plt = matplotlibcpp;

int main() {
  std::cout << "Saving plot to file..."; // << std::end;
  plt::plot({1,3,2,4});
  plt::save("plot.pdf");
}

