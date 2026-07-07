#include "Math.h"
#include "Vector3.h"
#include "Matrix3.h"
#include "Matrix4.h"
#include "Quaternion.h"

namespace BT_Geometry
{
Vector3 SphericalToCartesian(double latitude, double longitude, double radius)
{
	double radCosLat = radius * cos(latitude);

	return Vector3(
		radCosLat * cos(longitude),
		radCosLat * sin(longitude),
		radius * sin(latitude));
}

Vector3 CartesianToSpherical(double x, double y, double z)
{
	double radius = sqrt(x * x + y * y + z * z);
	double longitude = atan2(y, x);
	double latitude = asin(z / radius);

	return Vector3(latitude, longitude, radius);
}
}