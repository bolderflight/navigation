[![Pipeline](https://gitlab.com/bolderflight/software/navigation/badges/main/pipeline.svg)](https://gitlab.com/bolderflight/software/navigation/) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)  [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

![Bolder Flight Systems Logo](img/logo-words_75.png) &nbsp; &nbsp; ![Arduino Logo](img/arduino_logo_75.png)

# Navigation

   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description

# Installation

## Arduino
Simply clone or download and extract the zipped library into your Arduino/libraries folder. In addition to this library, the [Bolder Flight Systems Units library](https://github.com/bolderflight/units) and [Bolder Flight Systems Eigen library](https://github.com/bolderflight/eigen) must be installed. The library is added as:

```C++
#include "navigation.h"
```

An example is located in *examples/arduino/nav_example/nav_example.ino*. This library is tested with Teensy 3.x, 4.x, and LC devices and is expected to work with other Arduino ARM devices. It is **not** expected to work with AVR devices.

## CMake
CMake is used to build this library, which is exported as a library target called *navigation*. The header is added as:

```C++
#include "navigation.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a build directory and then, from within that directory issuing:

```
cmake ..
make
```

This will build the library, an example executable called *nav_example*, and an executable for testing using the Google Test framework, called *nav_test*. The example executable source file is located at *examples/cmake/nav_example.cc*.

# Namespace
This library is within the namespace *bfs*.

# Constants
The following constants are defined from WGS84:

| Description | Variable | Value |
| --- | --- | --- |
| Semi-major axis, m | double SEMI_MAJOR_AXIS_LENGTH_M | 6378137.0 |
| Semi-minor axis, m | double SEMI_MINOR_AXIS_LENGTH_M | 6356752.3142 |
| Flattening | double FLATTENING | 1.0 / 298.257223563 |
| First eccentricity | double ECC | 8.1819190842622e-2 |
| First eccentricity squared | double ECC2 | 6.69437999014e-3 |
| Angular velocity of the Earth, rad/s | WE_RADPS | 7292115.0e-11 |
| Earth's gravitational constant, m^3/s^2 | GM_M3PS2 | 3986004.418e8 |

# Filters

## Tilt Compass

**Eigen::Vector3f TiltCompass(const Eigen::Vector3f &accel, const Eigen::Vector3f &mag)** Estimates heading, pitch, and roll given 3-axis accelerometer and magnetometer data. Accelerometer and magnetometer data is expected to be oriented with the positive x-axis forward, y-axis out the right, and z-axis down. Heading, pitch, and roll are output in radians.

## 15 State EKF INS

# Transformations

## Position
These functions convert between:
   * Latitude, longitude, and altitude above the WGS84 ellipsoid (LLA)
   * Earth Centered Earth Fixed (ECEF)
   * Locally tangent North East Down (NED)

**Eigen::Vector3d lla2ecef(const Eigen::Vector3d &lla, const AngPosUnit ang = AngPosUnit::DEG)** Convert geodetic coordinates to Earth-centered Earth-fixed (ECEF) coordinates. Inputs are latitude, longitude, and altitude [m] above the WGS84 ellipsoid. Optionally, the units of latitude and longitude can be specified; by default, they are in degrees. Outputs are ECEF coordinates [m].

```C++
Eigen::Vector3d lla = {0, 45, 1000};
Eigen::Vector3d ecef = bfs::lla2ecef(lla);
```

**Eigen::Vector3d ecef2lla(const Eigen::Vector3d &ecef, const AngPosUnit ang = AngPosUnit::DEG)** Convert Earth-centered Earth-fixed (ECEF) coordinates to geodetic coordinates. Inputs are ECEF coordinates [m] and, optionally, the units for the output latitude and longitude; by default they are in degrees. Uses [Olson's method](https://ieeexplore.ieee.org/abstract/document/481290), which is iteration-free, very fast, and accurate.

```C++
Eigen::Vector3d ecef = {4510731, 4510731, 0};
Eigen::Vector3d lla = bfs::ecef2lla(ecef);
```

**Eigen::Vector3d ecef2ned(const Eigen::Vector3d &ecef, const Eigen::Vector3d &lla0, const AngPosUnit ang = AngPosUnit::DEG)** Rotates an Earth Centered Earth Fixed (ECEF) vector into a locally tangent North East Down (NED) frame. Inputs are the ECEF vector and the origin of the NED frame, given as latitude, longitude, and altitude [m] above the WGS84 ellipsoid. Optionally, the units of latitude and longitude can be specified; by default, they are in degrees.

```C++
Eigen::Vector3d ecef = {4510731, 4510731, 0};
Eigen::Vector3d lla0 = {0, 45, 999.956};
Eigen::Vector3d ned = bfs::ecef2ned(ecef, lla0);
```

**Eigen::Vector3d ned2ecef(const Eigen::Vector3d &ned, const Eigen::Vector3d &lla_ref, const AngPosUnit ang = AngPosUnit::DEG)** Rotates a North East Down (NED) vector into Earth Centered Earth Fixed (ECEF). Inputs are the NED vector and the origin of the NED frame, given as latitude, longitude, and altitude [m] above the WGS84 ellipsoid. Optionally, the units of latitude and longitude can be specified; by default, they are in degrees.

```C++
Eigen::Vector3d ned = {20, 20, 0};
Eigen::Vector3d lla0 = {0, 45, 999.956};
Eigen::Vector3d ecef = bfs::ned2ecef(ned, lla0);
```

**Eigen::Vector3d lla2ned(const Eigen::Vector3d &lla, const Eigen::Vector3d &lla0, const AngPosUnit ang = AngPosUnit::DEG)** Transforms geodetic coordinates to locally tangent North East Down (NED) coordinates. Inputs are the latitude, longitude, and altitude [m] above the WGS84 ellipsoid of the coordinates and latitude, longitude, and altitude [m] above the WGS84 ellipsoid of the origin of the NED frame. Optionally, the units of latitude and longitude can be specified; by default, they are in degrees.

```C++
Eigen::Vector3d lla0 = {46.017, 7.750, 1673};
Eigen::Vector3d lla = {45.976, 7.658, 4531};
Eigen::Vector3d ned = bfs::lla2ned(lla, lla0);
```

**Eigen::Vector3d ned2lla(const Eigen::Vector3d &ned, const Eigen::Vector3d &lla0, const AngPosUnit ang = AngPosUnit::DEG)** Transforms local North East Down (NED) coordinates to geodetic coordinates. Inputs are the local NED coordinates and the latitude, longitude, and altitude [m] above the WGS84 ellipsoid of the origin of the NED frame. Optionally, the units of latitude and longitude can be specified; by default, they are in degrees.

```C++
Eigen::Vector3d lla0 = {46.017, 7.750, 1673};
Eigen::Vector3d ned = {-4556.3, -7134.8, -2852.4};
Eigen::Vector3d lla = bfs::ned2lla(ned, lla0);
```

## Attitude
These functions convert between:
   * Rotation angle series
   * Euler angles ('ZYX' order rotation angles, i.e. yaw, pitch, roll)
   * Direction cosine matrix
   * Quaternions (i, j, k, w) 

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

# Earth Modeling
The following functions are converted from and tested against [NavPy](https://github.com/NavPy/NavPy). They compute the Earth's radius of curvature, LLA rate, Earth rotation rate resolved on a local NED frame, and the navigation rate.

**double earthrad_transverse_m(const double lat, const AngPosUnit ang = AngPosUnit::DEG)** Computes the radius of curvature [m] in the prime-vertical (i.e. transverse or east-west) direction given a latitude and, optionally, the latitude unit. If the latitude unit is not specified, it is in degrees by default.

**double earthrad_meridonal_m(const double lat, const AngPosUnit ang = AngPosUnit::DEG)** Computes the radius of curvature [m] in the meridonal (i.e. north-south) direction given a latitude and, optionally, the latitude unit. If the latitude unit is not specified, it is in degrees by default.

**std::array<double, 2> earthrad_m(const double lat, const AngPosUnit ang = AngPosUnit::DEG)** Computes the radius of curvature in the transverse and meridonal directions, returning the result as an array.

**Eigen::Vector3d llarate(const double vn, const double ve, const double vd, const double lat, const double alt, const AngPosUnit ang = AngPosUnit::DEG)** Computes the latitude, longitude, and altitude (LLA) rate given the locally tangent North-East-Down (NED) velocity [m/s] components, the latitude, altitude above the WGS84 ellipsoid [m], and, optionally, the latitude unit. If the latitude unit is not specified, it is in degrees by default. The latitude and longitude rate are given in deg/s if the latitude is given in degrees and rad/s if the latitude is given in radians.

**Eigen::Vector3f llarate(const Eigen::Vector3f &ned_vel, const Eigen::Vector3d &lla, const AngPosUnit ang = AngPosUnit::DEG)** Same functionality as the previous llarate function, but uses vectors for NED velocity and LLA input.

**Eigen::Vector3d earthrate(const double lat, const AngPosUnit ang = AngPosUnit::DEG)** Computes the Earth rotation rate resolved on the locally tangent North-East-Down (NED) frame, given a latitude and, optionally, the latitude unit. If the latitude unit is not specified, it is in degrees by default.

**Eigen::Vector3d navrate(const double vn, const double ve, const double vd, const double lat, const double alt, const AngPosUnit ang = AngPosUnit::DEG)** Computes the navigation/transport rate  given the locally tangent North-East-Down (NED) velocity [m/s] components, the latitude, altitude above the WGS84 ellipsoid [m], and, optionally, the latitude unit. If the latitude unit is not specified, it is in degrees by default. The navigation/transport rate is the angular velocity of the NED frame relative to the earth ECEF frame. The navigation rate is given in deg/s if the latitude is given in degrees and rad/s if the latitude is given in radians.

**Eigen::Vector3f navrate(const Eigen::Vector3f &ned_vel, const Eigen::Vector3d &lla, const AngPosUnit ang = AngPosUnit::DEG)** Same functionality as the previous navrate function, but uses vectors for NED velocity and LLA input.

# Utilities

**T WrapTo2Pi(T ang)** Converts a +/-PI radian angle to a 0 to 2PI angle. Templated by floating point type.

**T WrapToPi(T ang)** Converts a 0 to 2PI radian angle to a +/-PI angle. Templated by floating point type.

**Eigen::Matrix<T, 3, 3> Skew(const Eigen::Matrix<T, 3, 1> &w)** Creates a skew symmetric matrix from a vector. Templated by floating point type.
