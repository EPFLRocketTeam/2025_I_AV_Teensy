#include <imu_utils.h>

void print(const imu::Vector<3> &v, const char name[]) 
{
  Serial.print(name);
  Serial.print("X: ");
  Serial.print(v.x());
  Serial.print(" Y: ");
  Serial.print(v.y());
  Serial.print(" Z: ");
  Serial.print(v.z());
}

void print(const imu::Quaternion &q, const char name[])
{
  Serial.print(name);
  Serial.print("W: ");
  Serial.print(q.w());
  Serial.print(" X: ");
  Serial.print(q.x());
  Serial.print(" Y: ");
  Serial.print(q.y());
  Serial.print(" Z: ");
  Serial.print(q.z());
}

void print(const int8_t &i, const char name[])
{
  Serial.print(name);
  Serial.print(i);
}

void println(const imu::Vector<3> &v, const char name[]) 
{
  Serial.print(name);
  Serial.print("X: ");
  Serial.print(v.x());
  Serial.print(" Y: ");
  Serial.print(v.y());
  Serial.print(" Z: ");
  Serial.print(v.z());
  Serial.println();
}

void println(const imu::Quaternion &q, const char name[])
{
  Serial.print(name);
  Serial.print("W: ");
  Serial.print(q.w());
  Serial.print(" X: ");
  Serial.print(q.x());
  Serial.print(" Y: ");
  Serial.print(q.y());
  Serial.print(" Z: ");
  Serial.print(q.z());
  Serial.println();
}

void println(const int8_t &i, const char name[])
{
  Serial.print(name);
  Serial.print(i);
  Serial.println();
}
