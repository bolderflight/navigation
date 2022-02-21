[![Pipeline](https://gitlab.com/bolderflight/software/navigation/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/navigation/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Navigation

   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description

# Installation

## Arduino


## CMake

# Namespace
This library is within the namespace *bfs*.

# Transformations

## Attitude

**Eigen::Matrix<T, 3, 3> angle2dcm(const T rot1, const T rot2, const T rot3, const AngPosUnit ang = AngPosUnit::RAD)** Converts a series of rotation angles to a direction cosine matrix. Inputs are the three rotation angles, and optionally, the units of those angles. By default, angle input is expected in radians. Templated by floating point type.

```C++
float yaw = 0.7854;
float pitch = 0.1;
float roll = 0;
Eigen::Matrix3f dcm = bfs::angle2dcm(yaw, pitch, roll);
```

**Eigen::Matrix<T, 3, 3> eul2dcm(const Eigen::Matrix<T, 3, 1> &eul, const AngPosUnit ang = AngPosUnit::RAD)** Converts a given set of Euler angles to a direction cosine matrix. Input is the Euler angle vector (order 'ZYX') and optionally, the units of those angles. By default, angle input is expected in radians. Templated by floating point type.

```C++
float yaw = 0.7854;
float pitch = 0.1;
float roll = 0;
Eigen::Vector3f eul = {yaw, pitch, roll};
Eigen::Matrix3f dcm = bfs::eul2dcm(eul);
```

**Eigen::Matrix<T, 3, 1> dcm2angle(const Eigen::Matrix<T, 3, 3> &dcm, const AngPosUnit ang = AngPosUnit::RAD)** Creates a series of rotation angles from direction cosine matrix. Inputs are the direction cosine matrix, and optionally, the units of the output rotation angles. By default the angles are output in radians. Templated by floating point type.

```C++
Eigen::Matrix3f dcm;
dcm << 0.85253103550038,  0.47703040785184, -0.21361840626067,
      -0.43212157513194,  0.87319830445628,  0.22537893734811,
       0.29404383655186, -0.09983341664683,  0.95056378592206;
Eigen::Vector3f ang = bfs::dcm2angle(dcm);
```

**Eigen::Matrix<T, 3, 1> dcm2eul(const Eigen::Matrix<T, 3, 3> &dcm, const AngPosUnit ang = AngPosUnit::RAD)** Creates a set of Euler angles (order 'ZYX') from direction cosine matrix. Inputs are the direction cosine matrix, and optionally, the units of the output Euler angles. By default the angles are output in radians. Templated by floating point type. Functionally equivalent to *dcm2angle*.

```C++
Eigen::Matrix3f dcm;
dcm << 0.85253103550038,  0.47703040785184, -0.21361840626067,
      -0.43212157513194,  0.87319830445628,  0.22537893734811,
       0.29404383655186, -0.09983341664683,  0.95056378592206;
Eigen::Vector3f ang = bfs::dcm2eul(dcm);
```

**Eigen::Quaternion<T> angle2quat(const T rot1, const T rot2, const T rot3, const AngPosUnit a = AngPosUnit::RAD)** Convert rotation angles to quaternion. Inputs are the three rotation angles, and optionally, the units of those angles. By default, angle input is expected in radians. Templated by floating point type.

```C++
float yaw = 0.7854; 
float pitch = 0.1; 
float roll = 0;
Eigen::Quaternionf q = bfs::angle2quat(yaw, pitch, roll);
```

**Eigen::Quaternion<T> eul2quat(const T rot1, const T rot2, const T rot3, const AngPosUnit a = AngPosUnit::RAD)** Converts a given set of Euler angles to quaternion. Input is the Euler angle vector (order 'ZYX'), and optionally, the units of those angles. By default, angle input is expected in radians. Templated by floating point type.

```C++
float yaw = 0.7854; 
float pitch = 0.1; 
float roll = 0;
Eigen::Vector3f eul = {yaw, pitch, roll};
Eigen::Quaternionf q = bfs::eul2quat(eul);
```

**Eigen::Matrix<T, 3, 1> quat2angle(const Eigen::Quaternion<T> &q, const AngPosUnit ang = AngPosUnit::RAD)** Converts a quaternion to a series of rotation angles. Input is the quaternion, and optionally, the units of the output rotation angles. By default the angles are output in radians. Templated by floating point type.

```C++
Eigen::Quaternionf q;
q.x() = 0.215509;
q.y() = 0.432574;
q.z() = 0.0846792;
q.w() = 0.871358;
Eigen::Vector3f ang = bfs::quat2angle(q);
```

**Eigen::Matrix<T, 3, 1> quat2eul(const Eigen::Quaternion<T> &q, const AngPosUnit ang = AngPosUnit::RAD)** Converts a quaternion to Euler angles. Input is the quaternion, and optionally, the units of the output Euler angles. By default the angles are output in radians. Templated by floating point type. Functionally equivalent to *quat2angle*.

```C++
Eigen::Quaternionf q;
q.x() = 0.215509;
q.y() = 0.432574;
q.z() = 0.0846792;
q.w() = 0.871358;
Eigen::Vector3f ang = bfs::quat2eul(q);
```

**Eigen::Quaternion<T> dcm2quat(const Eigen::Matrix<T, 3, 3> &dcm)** Convert direction cosine matrix to quaternion. Input is the direction cosine matrix and output is the quaternion. Templated by floating point type.

```C++
Eigen::Matrix3f dcm;
dcm << 0.4330,  0.2500, -0.8660,
       0.1768,  0.9186,  0.3536,
       0.8839, -0.3062,  0.3536;
Eigen::Quaternionf q = bfs::dcm2quat(dcm);
```

**Eigen::Matrix<T, 3, 3> quat2dcm(const Eigen::Quaternion<T> &q)** Convert quaternion to direction cosine matrix. Input is the quaternion and output is the direction cosine matrix. Templated by floating point type.

```C++
Eigen::Quaternionf q;
q.x() = 0.200578;
q.y() = 0.531966;
q.z() = 0.0222526;
q.w() = 0.822375;
Eigen::Matrix3f dcm = bfs::quat2dcm(q);
```
