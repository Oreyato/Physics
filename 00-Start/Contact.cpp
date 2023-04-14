#include "Contact.h"


void Contact::ResolveContact(Contact& contact)
{
	Body* a = contact.a;
	Body* b = contact.b;

	a->linearVelocity.Zero();
	b->linearVelocity.Zero();
}
