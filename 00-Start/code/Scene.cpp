//
//  Scene.cpp
//
#include "Scene.h"
#include "../Shape.h"
#include "../Intersections.h"
#include "../Contact.h"


/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < bodies.size(); i++ ) {
		delete bodies[ i ].shape;
	}
	bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	// -- BODIES --
	// Ball
	Body ball;
	ball.position = Vec3( 0, 0, 10 );
	ball.orientation = Quat( 0, 0, 0, 1 );
	ball.shape = new ShapeSphere( 1.0f );
	ball.inverseMass = 1.0f;
	ball.elasticity = 0.5f;
	ball.friction = 0.5f;
	ball.linearVelocity = Vec3(1, 0, 0);

	bodies.push_back(ball);

	// -- GROUND --
	Body earth;
	earth.position = Vec3(0, 0, -1000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(1000.0f);
	earth.inverseMass = 0.0f;
	earth.elasticity = 1.0f;
	earth.friction = 0.5f;

	bodies.push_back(earth);
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	// -- GRAVITY --
	for (int i = 0; i < bodies.size(); i++) 
	{
		Body& body = bodies[i];

		float mass = 1.0f / body.inverseMass;

		// Gravity needs to be an impulse I
		// I == dp, so F == dp/dt <=> dp = F * dt
		// <=> I = F * dt <=> I = m * g * dt
		Vec3 impulseGravity = Vec3(0, 0, - GRAVITY_AMOUNT) * mass * dt_sec;
		body.ApplyImpulseLinear(impulseGravity);
	}

	//v Collisions check =============================================
	int numContacts = 0;
	const int maxContacts = bodies.size() * bodies.size();
	Contact* contacts = (Contact*)_malloca(sizeof(Contact) * maxContacts);

	for (int i = 0; i < bodies.size(); i++) {
		for (int j = i + 1; j < bodies.size(); j++) {
			Body& bodyA = bodies[i];
			Body& bodyB = bodies[j];

			// Ignore collisions for bodies with infinite mass
			if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f) continue;

			Contact contact;
			if (Intersections::Intersect(bodyA, bodyB, dt_sec, contact)) {
				contacts[numContacts] = contact;
				++numContacts;
			}
		}
	}

	// Sort times of impact
	if (numContacts > 1) {
		qsort(contacts, numContacts, sizeof(Contact), Contact::CompareContact);
	}

	// Contact resolve in order
	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; ++i)
	{
		Contact& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;
		Body* bodyA = contact.a;
		Body* bodyB = contact.b;
		
		// Skip body with infinite mass
		if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f) continue;

		// Update position
		for (int j = 0; j < bodies.size(); ++j) {
			bodies[j].Update(dt);
		}

		Contact::ResolveContact(contact);
		accumulatedTime += dt;
	}
	//^ Collisions check =============================================

	// Other physics behaviours, outside collisions
	// Update the positions for the rest of this frame's time
	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		// Position update
		for (int i = 0; i < bodies.size(); ++i) {
			bodies[i].Update(timeRemaining);
		}
	}

}