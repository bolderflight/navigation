/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef NAVIGATION_SRC_UTILS_H_  // NOLINT
#define NAVIGATION_SRC_UTILS_H_

#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {

/* Converts a +/- 180 value to a 0 - 360 value */
template<typename T>
T WrapTo2Pi(T ang) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  ang = std::fmod(ang, BFS_2PI<T>);
  if (ang < static_cast<T>(0)) {
    ang += BFS_2PI<T>;
  }
  return ang;
}

/* Converts a 0 - 360 value to a +/- 180 value */
template<typename T>
T WrapToPi(T ang) {
  static_assert(std::is_floating_point<T>::value,
                "Only floating point types supported");
  if (ang > BFS_PI<T>) {
    ang -= BFS_2PI<T>;
  }
  if (ang < -BFS_PI<T>) {
    ang += BFS_2PI<T>;
  }
  return ang;
}

/* Skew symmetric matrix from a given vector w */
template<typename T>
Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &w) {
  Eigen::Matrix<T, 3, 3> c;
  c(0, 0) =  0.0;
  c(0, 1) = -w(2);
  c(0, 2) =  w(1);
  c(1, 0) =  w(2);
  c(1, 1) =  0.0;
  c(1, 2) = -w(0);
  c(2, 0) = -w(1);
  c(2, 1) =  w(0);
  c(2, 2) =  0.0;
  return c;
}

}  // namespace bfs

#endif  // NAVIGATION_SRC_UTILS_H_ NOLINT
