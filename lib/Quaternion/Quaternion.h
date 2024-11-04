/*
 * Quaternion and Vector3 math library,
 * includes operations for ZYX Euler Angles, rate integration and much more
 *
 * O. Rangel Morales
 * (GitHub: OrlandoR4)
 * MIT license, all text above must be included in any redistribution
 */

#pragma once

/**
 * Class to perform 3 dimensional vector math, also used for quaternions
 *
 * x - y - z components
 *
 * X Roll Y Pitch Z Yaw
 */
class Vector3
{
public:
  float x;
  float y;
  float z;

  Vector3()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  Vector3(float x_, float y_, float z_)
  {
    x = x_;
    y = y_;
    z = z_;
  }

  // Operators

  Vector3 operator+(Vector3 vt);
  Vector3 operator+(float nt);
  Vector3 operator-(Vector3 vt);
  Vector3 operator-(float nt);
  Vector3 operator+=(Vector3 vt);
  Vector3 operator+=(float nt);
  Vector3 operator-=(Vector3 vt);
  Vector3 operator-=(float nt);
  Vector3 operator*(Vector3 vt);
  Vector3 operator*(float nt);
  Vector3 operator/(Vector3 vt);
  Vector3 operator/(float nt);
  Vector3 operator*=(Vector3 vt);
  Vector3 operator*=(float nt);
  Vector3 operator/=(Vector3 vt);
  Vector3 operator/=(float nt);

  Vector3 normalize();
  float get_magnitude();

  float dot_product(Vector3 vt);
  Vector3 cross_product(Vector3 vt);

  float angle_between_vectors(Vector3 vt);

  // Representation Operations

  Vector3 rad_to_deg();
  Vector3 deg_to_rad();
};

/**
 * Class to perform quaternion math
 *
 * w - x - y - z components
 *
 * Input for euler angle and other orientation math: X Roll Y Pitch Z Yaw
 * Euler rotation order: Z - Y - X
 */
class Quaternion
{
public:
  float w;
  float x;
  float y;
  float z;

  Quaternion()
  {
    w = 1.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
  Quaternion(float w_, float x_, float y_, float z_)
  {
    w = w_;
    x = x_;
    y = y_;
    z = z_;
  }

  // Operators

  Quaternion operator*(Quaternion qt);
  Quaternion operator*=(Quaternion qt);

  Quaternion normalize();
  Quaternion conjugate();

  // Vector Operations

  Vector3 rotate_vector(Vector3 vt);
  Quaternion from_axis_angle(float theta, Vector3 vt);

  // Representation Operations

  Vector3 quaternion_to_euler();
  Quaternion euler_to_quaternion(Vector3 vt);

  // State Updates

  Quaternion update_with_rates(float dt, Vector3 rates);
  Quaternion update_with_accel(Vector3 at, Vector3 target_vector, float gain);
  Quaternion update_with_mag(Vector3 mt, Vector3 at, Vector3 target_vector, Quaternion declination_compensation, float gain);
};