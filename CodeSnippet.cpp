#include <cmath>

namespace physics
{
	struct Vector3
	{
		Vector3()
			: x(0.), y(0.), z(0.)
		{}
		Vector3( const double _x, const double _y, const double _z )
			: x( _x ), y( _y ), z( _z )
		{}
		static Vector3 Zero()
		{
			return Vector3(0., 0., 0.);
		}
		double x, y, z;
	};
	const double ToRadian = DirectX::XM_PI/180.;
	double Time = 0.;
	double DeltaTime = 0.01;
	double AccelerationOfGravity = 9.8;
	double DragCoefficient = 1;
	double ResistanceCoefficientOfWind = 1;
////
//
////
	double ThetaXZtoY = 0.;
	double ThetaXto_Z = -45.;
	double Length = 1;
	double MuzzleVelocity = 10.;
	double BaseY = 0.; // Height
	Vector3 _VLength, _VMuzzleVelocity;
	double ProjectileMass = 10;
	double WindVelocity = 1;
	double WindThetaXto_Z = 0;
	Vector3 _VWindVelocity;
}

void init()
{
////
//
////
	const double xz = physics::Length * std::cos(physics::ThetaXZtoY * physics::ToRadian);
	physics::_VLength.x = xz * cos(physics::ThetaXto_Z * physics::ToRadian);
	physics::_VLength.z = xz * sin(physics::ThetaXto_Z * physics::ToRadian);
	const double bodyProjY = std::cos((90.-physics::ThetaXZtoY) * physics::ToRadian);
	physics::_VLength.y = physics::Length * bodyProjY;
	physics::_VMuzzleVelocity.x = physics::_VLength.x / physics::Length * physics::MuzzleVelocity;
	physics::_VMuzzleVelocity.z = physics::_VLength.z / physics::Length * physics::MuzzleVelocity;
	physics::_VMuzzleVelocity.y = bodyProjY * physics::MuzzleVelocity;
	physics::_VWindVelocity.x = physics::WindVelocity * std::cos(physics::WindThetaXto_Z*physics::ToRadian);
	physics::_VWindVelocity.z = physics::WindVelocity * std::sin(physics::WindThetaXto_Z*physics::ToRadian);
}

void simulate()
{
	using namespace physics;
	Time += DeltaTime;

	Vector3& vMuzzle = _VLength;
	DirectX::XMFLOAT3& position = mSphere->mTransformation.position;
	position.x = (float)((_VMuzzleVelocity.x * Time) + vMuzzle.x);
	position.z = (float)((_VMuzzleVelocity.z * Time) + vMuzzle.z);
	position.y = (float)(BaseY + vMuzzle.y + (_VMuzzleVelocity.y * Time) - (0.5 * AccelerationOfGravity * Time * Time));
	const double massOverDragCoef = ProjectileMass/DragCoefficient;
	const double temp = std::exp(-Time/massOverDragCoef);
	const double compWindForceOverDragCoef = ResistanceCoefficientOfWind/DragCoefficient;
	const double windForceXOverDragCoef = compWindForceOverDragCoef*_VWindVelocity.x;
	const double tempX = massOverDragCoef*(-windForceXOverDragCoef-_VMuzzleVelocity.x);
	position.x += (float)(temp*tempX - tempX - windForceXOverDragCoef*Time);
	const double windForceZOverDragCoef = compWindForceOverDragCoef*_VWindVelocity.z;
	const double tempZ = massOverDragCoef*(-windForceZOverDragCoef-_VMuzzleVelocity.z);
	position.z += (float)(temp*tempZ - tempZ - windForceZOverDragCoef*Time);
	const double gForceYOverDragCoef = massOverDragCoef*AccelerationOfGravity;
	const double tempY = massOverDragCoef*(gForceYOverDragCoef+_VMuzzleVelocity.y);
	position.y += (float)(-temp*tempY + tempY - gForceYOverDragCoef*Time);
}