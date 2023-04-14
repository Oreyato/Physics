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
	bodies.push_back(ball);

	// -- GROUND --
	Body earth;
	earth.position = Vec3(0, 0, -1000);
	earth.orientation = Quat(0, 0, 0, 1);
	earth.shape = new ShapeSphere(1000.0f);
	earth.inverseMass = 0.0f;
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
		body.AddImpulseLinear(impulseGravity);
	}

	// -- COLLISIONS CHECK --
	for (int i = 0; i < bodies.size(); i++) {
		for (int j = i + 1; j < bodies.size(); j++) {
			Body& bodyA = bodies[i];
			Body& bodyB = bodies[j];

			// Ignore collisions for bodies with infinite mass
			if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f) continue;

			Contact contact;
			if (Intersections::Intersect(bodyA, bodyB, contact)) {
				Contact::ResolveContact(contact);
			}
		}
	}

	// -- POSITION UPDATE --
	for (int i = 0; i < bodies.size(); i++)
	{
		// Linear velocity
		bodies[i].position += bodies[i].linearVelocity * dt_sec;
	}
}