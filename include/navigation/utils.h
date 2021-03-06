/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#ifndef INCLUDE_NAVIGATION_UTILS_H_
#define INCLUDE_NAVIGATION_UTILS_H_

#include "Eigen/Core"

namespace bfs {
/* Rate of change of LLA given NED velocity */
Eigen::Vector3f LlaRate(const Eigen::Vector3f &ned_vel,
                        const Eigen::Vector3d &lla);
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

#endif  // INCLUDE_NAVIGATION_UTILS_H_
