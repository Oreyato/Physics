#include "Intersections.h"


bool Intersections::Intersect(Body& a, Body& b)
{
	const Vec3 ab = b.position - a.position;

	//v Spheres collisions ===========================================
	if (a.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE &&
		b.shape->GetType() == Shape::ShapeType::SHAPE_SPHERE) {

		ShapeSphere* sphereA = reinterpret_cast<ShapeSphere*>(a.shape);
		ShapeSphere* sphereB = reinterpret_cast<ShapeSphere*>(b.shape);

		const float radiusAB = sphereA->radius + sphereB->radius;

		if (ab.GetLengthSqr() < radiusAB * radiusAB) {
			return true;
		}
	}
	//^ Spheres collisions ===========================================

	return false;
}
