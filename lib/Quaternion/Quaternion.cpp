/*
 * Quaternion and Vector3 math library,
 * includes operations for ZYX Euler Angles, rate integration and much more
 *
 * O. Rangel Morales
 * (GitHub: OrlandoR4)
 * MIT license, all text above must be included in any redistribution
 */

#include <Arduino.h>
#include "Quaternion.h"

// Vector3 Functions

Vector3 Vector3::operator+(Vector3 vt)
{
  float xn = x + vt.x;
  float yn = y + vt.y;
  float zn = z + vt.z;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator+(float nt)
{
  float xn = x + nt;
  float yn = y + nt;
  float zn = z + nt;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator+=(Vector3 vt)
{
  x = x + vt.x;
  y = y + vt.y;
  z = z + vt.z;

  return *this;
}
Vector3 Vector3::operator+=(float nt)
{
  x = x + nt;
  y = y + nt;
  z = z + nt;

  return *this;
}

Vector3 Vector3::operator-(Vector3 vt)
{
  float xn = x - vt.x;
  float yn = y - vt.y;
  float zn = z - vt.z;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator-(float nt)
{
  float xn = x - nt;
  float yn = y - nt;
  float zn = z - nt;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator-=(Vector3 vt)
{
  x = x - vt.x;
  y = y - vt.y;
  z = z - vt.z;

  return *this;
}
Vector3 Vector3::operator-=(float nt)
{
  x = x - nt;
  y = y - nt;
  z = z - nt;

  return *this;
}

Vector3 Vector3::operator*(Vector3 vt)
{
  float xn = x * vt.x;
  float yn = y * vt.y;
  float zn = z * vt.z;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator*(float nt)
{
  float xn = x * nt;
  float yn = y * nt;
  float zn = z * nt;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator*=(Vector3 vt)
{
  x = x * vt.x;
  y = y * vt.y;
  z = z * vt.z;

  return *this;
}
Vector3 Vector3::operator*=(float nt)
{
  x = x * nt;
  y = y * nt;
  z = z * nt;

  return *this;
}

Vector3 Vector3::operator/(Vector3 vt)
{
  float xn = x / vt.x;
  float yn = y / vt.y;
  float zn = z / vt.z;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator/(float nt)
{
  float xn = x / nt;
  float yn = y / nt;
  float zn = z / nt;

  return Vector3(xn, yn, zn);
}
Vector3 Vector3::operator/=(Vector3 vt)
{
  x = x / vt.x;
  y = y / vt.y;
  z = z / vt.z;

  return *this;
}
Vector3 Vector3::operator/=(float nt)
{
  x = x / nt;
  y = y / nt;
  z = z / nt;

  return *this;
}

/**
 * Returns normalized vector (scales length to 1) if its length is not zero
 * @return Normalized vector
 */
Vector3 Vector3::normalize()
{
  float norm = sqrtf(x * x + y * y + z * z);

  if (norm != 0)
  {
    return Vector3(x / norm,
                   y / norm,
                   z / norm);
  }

  return *this;
}
/**
 * Returns length of Vector3
 * @return Length of Vector3
 */
float Vector3::get_magnitude()
{
  return sqrtf(x * x + y * y + z * z);
}
/**
 * Dot product of this vector and another vector vt
 * @param vt Other vector to calculate dot product with
 * @return Result of dot product
 */
float Vector3::dot_product(Vector3 vt)
{
  float xn = x * vt.x;
  float yn = y * vt.y;
  float zn = z * vt.z;

  return (xn + yn + zn);
}
/**
 * Cross product of this vector and another vector vt (v_this x v_other)
 * @param vt Other vector to calculate cross product with
 * @return Result of cross product
 */
Vector3 Vector3::cross_product(Vector3 vt)
{
  float xn = (y * vt.z) - (z * vt.y);
  float yn = -((x * vt.z) - (z * vt.x));
  float zn = (x * vt.y) - (y * vt.x);

  return Vector3(xn, yn, zn);
}
/**
 * Returns the angle between the two vectors
 * 
 * @param vt Other vector
 * @return Angle between the vectors 
 */
float Vector3::angle_between_vectors(Vector3 vt)
{
  float magnitude_product = get_magnitude() * vt.get_magnitude();

  if (magnitude_product != 0)
      return acosf(dot_product(vt) / magnitude_product);
  
  return 0;
}

/**
 * Returns conversion of Vector3 xyz radian components to degrees
 * @return Converted Vector3
 */
Vector3 Vector3::rad_to_deg()
{
  float xn = x * 180.0 / 3.14159;
  float yn = y * 180.0 / 3.14159;
  float zn = z * 180.0 / 3.14159;

  return Vector3(xn, yn, zn);
}
/**
 * Returns conversion of Vector3 xyz degree components to radians
 * @return Converted Vector3
 */
Vector3 Vector3::deg_to_rad()
{
  float xn = x / 180.0 * 3.14159;
  float yn = y / 180.0 * 3.14159;
  float zn = z / 180.0 * 3.14159;

  return Vector3(xn, yn, zn);
}

// Quaternion Functions 

/**
 * Quaternion product operator
 * @param qt Other quaternion to do the product with
 * @return This quaternion
 */
Quaternion Quaternion::operator*(Quaternion qt)
{
  float wn = (w * qt.w) - (x * qt.x) - (y * qt.y) - (z * qt.z);
  float xn = (w * qt.x) + (x * qt.w) + (y * qt.z) - (z * qt.y);
  float yn = (w * qt.y) - (x * qt.z) + (y * qt.w) + (z * qt.x);
  float zn = (w * qt.z) + (x * qt.y) - (y * qt.x) + (z * qt.w);

  return Quaternion(wn, xn, yn, zn);
}
/**
 * Quaternion product operator, updates the current quaternion
 * @param qt Other quaternion to do the product with
 * @return This quaternion
 */
Quaternion Quaternion::operator*=(Quaternion qt)
{
  w = (w * qt.w) - (x * qt.x) - (y * qt.y) - (z * qt.z);
  x = (w * qt.x) + (x * qt.w) + (y * qt.z) - (z * qt.y);
  y = (w * qt.y) - (x * qt.z) + (y * qt.w) + (z * qt.x);
  z = (w * qt.z) + (x * qt.y) - (y * qt.x) + (z * qt.w);

  return *this;
}

/**
 * Returns normalized quaternion (sets length to 1) if its magnitude is not zero
 * @return Normalized quaternion
 */
Quaternion Quaternion::normalize()
{
  float norm = sqrtf(w * w + x * x + y * y + z * z);

  if (norm != 0)
  {
    return Quaternion(w / norm,
                      x / norm,
                      y / norm,
                      z / norm);
  }

  return *this;
}
/**
 * Calculates conjugate of quaternion (backwards rotation)
 * @return Conjugate of this quaternion
 */
Quaternion Quaternion::conjugate()
{
  float wn = w;
  float xn = -x;
  float yn = -y;
  float zn = -z;

  return Quaternion(wn, xn, yn, zn);
}
/**
 * Returns Vector3 rotated by a quaternion
 * @param vt Vector3 to be rotated
 * @return vt after rotation
 */
Vector3 Quaternion::rotate_vector(Vector3 vt)
{
  // Normal Vector Rotation
  /*Quaternion vector_quat = *this * Quaternion(0.0f, vt.x, vt.y, vt.z) * conjugate();

  return Vector3(vector_quat.x, vector_quat.y, vector_quat.z);*/

  // Faster Vector Rotation https://www.johndcook.com/blog/2021/06/16/faster-quaternion-rotations/
  Vector3 qv = Vector3(x, y, z);
  Vector3 t = (qv * 2.0).cross_product(vt);
  vt += t * w + qv.cross_product(t);

  return vt;
}
/**
 * Sets quaternion from axis-angle representation
 * @param theta Angle component
 * @param vt Vector component
 * @return This quaternion after conversion
 */
Quaternion Quaternion::from_axis_angle(float theta, Vector3 vt)
{
  w = cosf(theta * 0.5);

  float sn = sinf(theta * 0.5);
  x = vt.x * sn;
  y = vt.y * sn;
  z = vt.z * sn;

  return *this;
}

/**
 * Calculates euler representation of quaternion rotation
 * (Z-Y-X Yaw-Pitch-Roll)
 * @return Euler angles vector (Roll - Pitch - Yaw)
 */
Vector3 Quaternion::quaternion_to_euler()
{
  float roll = atan2f(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  float pitch = 2.0 * (w * y - z * x);
  float yaw = atan2f(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  return Vector3(roll, pitch, yaw); // Z-Y-X Euler Angles (yaw-pitch-roll)
}
/**
 * Converts XYZ Euler angles to a quaternion representation
 * (Z-Y-X Yaw-Pitch-Roll order)
 * @param vt Euler angles vector (Roll - Pitch - Yaw)
 * @return This quaternion after conversion
 */
Quaternion Quaternion::euler_to_quaternion(Vector3 vt)
{
  float cy = cosf(vt.z * 0.5);
  float sy = sinf(vt.z * 0.5);
  float cp = cosf(vt.y * 0.5);
  float sp = sinf(vt.y * 0.5);
  float cr = cosf(vt.x * 0.5);
  float sr = sinf(vt.x * 0.5);

  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;

  return *this;
}

/**
 * Updates quaternion from body-axis angular rates
 * @param rates Radian angular rates
 * @param dt Delta time
 * @return This quaternion after updating
 */
Quaternion Quaternion::update_with_rates(float dt, Vector3 rates)
{
  float angle = rates.get_magnitude();

  if (angle == 0)
    return *this;

  rates /= angle;
  *this *= Quaternion().from_axis_angle(angle * dt, rates);
  *this = normalize();

  return *this;
}
/**
 * Updates quaternion from inertial-frame accelerometer readings, (x = +1g | +9.8m/s^2).
 * Algorithm works on the idea that the x axis (upwards) should have all the acceleration while y and z should be zero
 *
 * @param vt Dimensionless inertial accelerometer measurements
 * @param target_vector Vector to which the accelerometers should "point at"
 * @param gain How much to rotate quaternion by (0 - 1.0f)
 * @return This quaternion after updating
 */
Quaternion Quaternion::update_with_accel(Vector3 at, Vector3 target_vector, float gain)
{
  // Get normalized acceleration vector
  float at_magnitude = at.get_magnitude();

  if (at_magnitude == 0)
    return *this;

  at /= at_magnitude;

  // Find angle between ideal gravity and measured gravity
  float theta = acosf(at.dot_product(target_vector));

  // Find axis of the rotation between ideal gravity and measured gravity
  Vector3 rot_axis = at.cross_product(target_vector);

  // Rotate orientation towards ideal gravity with a gain
  *this = Quaternion().from_axis_angle(theta * gain, rot_axis) * (*this);

  // Normalize quaternion
  *this = normalize();

  return *this;
}
/**
 * Updates quaternion from body-frame magnetometer readings in the same manner the accelerometer update does
 *
 * @param mt Dimensionless body magnetometer measurements
 * @param at Dimensionless body accelerometer measurements
 * @param target_vector Vector to which EAST should "point at"
 * @param gain How much to rotate quaternion by (0 - 1.0f)
 * @return This quaternion after updating
 */
Quaternion Quaternion::update_with_mag(Vector3 mt, Vector3 at, Vector3 target_vector, Quaternion declination_compensation, float gain)
{

  float mt_magnitude = mt.get_magnitude();
  float at_magnitude = at.get_magnitude();

  if ((mt_magnitude == 0) || (at_magnitude == 0))
    return *this;

  // Get normalized magnetometer body axis readings
  Vector3 mag_unit_v = mt / mt_magnitude;

  // Get normalized "gravity" body axis readings
  Vector3 down_v = (at / at_magnitude) * -1.0;

  // Find body axis "east" from magnetometer and accelerometer vectors then rotate to inertial frame
  Vector3 inertial_east_v = rotate_vector(down_v.cross_product(mag_unit_v).normalize());

  // Compensate for declination
  inertial_east_v = declination_compensation.rotate_vector(inertial_east_v);

  // Rotate orientation to satisfy inertial axis "east" to true east
  update_with_accel(inertial_east_v, target_vector, gain);

  return *this;
}

/*
Notes

float cs = cosf(theta);
float sn = sinf(theta);

float rotated_x = x * cs - y * sn;
float rotated_y = x * sn + y * cs;
*/