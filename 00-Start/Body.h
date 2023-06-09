#pragma once
#include "code/Math/Vector.h"
#include "code/Renderer/model.h"
#include "code/Math/Quat.h"

class Body
{
public:
	Vec3 position;
	Quat orientation;
	float inverseMass;
	float elasticity;
	float friction;

	Vec3 linearVelocity;
	Vec3 angularVelocity;

	Shape* shape;

	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassBodySpace() const;

	Vec3 WorldSpaceToBodySpace(const Vec3& worldPoint);
	Vec3 BodySpaceToWorldSpace(const Vec3& bodyPoint);

	void ApplyImpulseLinear(const Vec3& impulse);
	void ApplyImpulseAngular(const Vec3& impulse);
	void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);

	Mat3 GetInverseInertiaTensorBodySpace() const;
	Mat3 GetInverseInertiaTensorWorldSpace() const;

	void Update(const float dt_sec);
};

