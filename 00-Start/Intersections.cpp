#include "Intersections.h"


bool Intersections::Intersect(Body& a, Body& b, Contact& contact)
{
	contact.a = &a;
	contact.b = &b;

	const Vec3 ab = b.position - a.position;

	contact.normal = ab;
	contact.normal.Normalize();

	//v Spheres collisions ===========================================
	if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE &&
		b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) {

		ShapeSphere* sphereA = reinterpret_cast<ShapeSphere*>(a.shape);
		ShapeSphere* sphereB = reinterpret_cast<ShapeSphere*>(b.shape);

		contact.ptOnAWorldSpace = a.position + contact.normal * sphereA->radius;
		contact.ptOnBWorldSpace = b.position - contact.normal * sphereB->radius;

		const float radiusAB = sphereA->radius + sphereB->radius;

		if (ab.GetLengthSqr() < radiusAB * radiusAB) {
			return true;
		}
	}
	//^ Spheres collisions ===========================================

	return false;
}
