#include "Contact.h"


void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;

	const float invMassA = a->inverseMass;
	const float invMassB = b->inverseMass;

	//v Collision impulse ============================================
	const Vec3& n = contact.normal;
	const Vec3& velAB = a->linearVelocity - b->linearVelocity;
	const float impulseValueJ = -2.0f * velAB.Dot(n) / (invMassA + invMassB);
	const Vec3 impulse = n * impulseValueJ;

	a->AddImpulseLinear(impulse);
	b->AddImpulseLinear(impulse * -1.0f);
	//^ Collision impulse ============================================

	// If object are interpenetrating, use this to set them on contact
	const float tA = invMassA / (invMassA + invMassB);
	const float tB = invMassB / (invMassA + invMassB);
	const Vec3 d = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;

	a->position += d * tA;
	b->position -= d * tB;
}
