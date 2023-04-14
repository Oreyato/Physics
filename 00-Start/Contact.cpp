#include "Contact.h"


void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;

	a->linearVelocity.Zero();
	b->linearVelocity.Zero();

	// If object are interpenetrating, use this to set them on contact
	const float tA = a->inverseMass / (a->inverseMass + b->inverseMass);
	const float tB = b->inverseMass / (a->inverseMass + b->inverseMass);
	const Vec3 d = contact.ptOnBWorldSpace - contact.ptOnAWorldSpace;

	a->position += d * tA;
	b->position -= d * tB;
}
