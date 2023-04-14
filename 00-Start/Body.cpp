#include "Body.h"
#include "Shape.h"


Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = shape->GetCenterOfMass();
	const Vec3 pos = position + orientation.RotatePoint(centerOfMass);

	return pos;
}

Vec3 Body::GetCenterOfMassBodySpace() const
{
	return shape->GetCenterOfMass();
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPoint)
{
	const Vec3 temp = worldPoint - GetCenterOfMassWorldSpace();
	const Quat invertOrient = orientation.Inverse();

	Vec3 bodySpace = invertOrient.RotatePoint(temp);

	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& bodyPoint)
{
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + orientation.RotatePoint(bodyPoint);

	return worldSpace;
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	// dv = J / m
	linearVelocity += impulse * inverseMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	// L = I w = r x p
	// dL = I dw = r x J
	// dw = I^-1 * ( r x J )
	angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	// Clamp angular velocity - for performances reasons
	// -- 30 rad per seconds, sufficient for now
	const float maxAngularSpeed = 30.0f;

	if (angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
	{
		angularVelocity.Normalize();
		angularVelocity *= maxAngularSpeed;
	}
}

/// <summary>
/// Apply impulse on a specific world space
/// </summary>
/// <param name="impulsePoint">
/// The world space location of the application of the impulse
/// </param>
/// <param name="impulse">
/// The world space direction and magnitude of the impulse
///</param>
void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (inverseMass == 0.0f) return;

	ApplyImpulseLinear(impulse);

	// Applying impulse must produce torques through the center of mass
	Vec3 position = GetCenterOfMassWorldSpace();
	Vec3 r = impulsePoint - position;
	Vec3 dL = r.Cross(impulse); // World space
	ApplyImpulseAngular(dL);
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertiaTensor = shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;

	return inverseInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = shape->InertiaTensor();
	Mat3 inverseInertiaTensor = inertiaTensor.Inverse() * inverseMass;
	Mat3 orient = orientation.ToMat3();

	inverseInertiaTensor = orient * inverseInertiaTensor * orient.Transpose();

	return inverseInertiaTensor;
}

void Body::Update(const float dt_sec)
{
	position += linearVelocity * dt_sec;

	// We have an angular velocity around the center of mass,
	// this needs to be converted relative to model position.
	// This way we can properly update the orientation of the model
	Vec3 positionCM = GetCenterOfMassWorldSpace();
	Vec3 CMToPositon = position - positionCM;

	// Total torques is equal to external applied torques + internal torque (precession)
	// T = Texternal + w x I * w
	// Texternal = 0 because it was applied in the collision response function
	// T = Ia = w x I * w
	// a = I^-1 (w x I * w)
	Mat3 orientationMat = orientation.ToMat3();
	Mat3 inertiaTensor = orientationMat	* shape->InertiaTensor() * orientationMat.Transpose();
	Vec3 alpha = inertiaTensor.Inverse() * (angularVelocity.Cross(inertiaTensor * angularVelocity));
	angularVelocity += alpha * dt_sec;

	// Update orientation
	Vec3 dAngle = angularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	orientation = dq * orientation;
	orientation.Normalize();

	// Get the new model position
	position = positionCM + dq.RotatePoint(CMToPositon);
}
