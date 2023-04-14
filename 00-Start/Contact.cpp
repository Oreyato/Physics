#include "Contact.h"


void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;

	const float invMassA = a->inverseMass;
	const float invMassB = b->inverseMass;

	const float elasticityA = a->elasticity;
	const float elasticityB = b->elasticity;
	const float elasticity = elasticityA * elasticityB;

	const Vec3 ptOnA = contact.ptOnAWorldSpace;
	const Vec3 ptOnB = contact.ptOnBWorldSpace;

	const Mat3 inverseWorldInertiaA = a->GetInverseInertiaTensorWorldSpace();
	const Mat3 inverseWorldInertiaB = b->GetInverseInertiaTensorWorldSpace();
	const Vec3 n = contact.normal;
	const Vec3 rA = ptOnA - a->GetCenterOfMassWorldSpace();
	const Vec3 rB = ptOnB - b->GetCenterOfMassWorldSpace();

	const Vec3 angularJA = (inverseWorldInertiaA * rA.Cross(n)).Cross(rA);
	const Vec3 angularJB = (inverseWorldInertiaB * rB.Cross(n)).Cross(rB);
	const float angularFactor = (angularJA + angularJB).Dot(n);

	// Get world space velocity of the motion and rotation
	const Vec3 velA = a->linearVelocity + a->angularVelocity.Cross(rA);
	const Vec3 velB = b->linearVelocity + b->angularVelocity.Cross(rB);

	//v Collision impulse ============================================
	const Vec3& velAB = velA - velB;
	const float impulseValueJ = (1.0f + elasticity) * velAB.Dot(n) / (invMassA + invMassB + angularFactor);
	const Vec3 impulse = n * impulseValueJ;

	a->ApplyImpulse(ptOnA, impulse * -1.0f); 
	b->ApplyImpulse(ptOnB, impulse * 1.0f);
	//^ Collision impulse ============================================

	// If object are interpenetrating, use this to set them on contact
	const float tA = invMassA / (invMassA + invMassB);
	const float tB = invMassB / (invMassA + invMassB);
	const Vec3 d = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;

	a->position += d * tA;
	b->position -= d * tB;
}
