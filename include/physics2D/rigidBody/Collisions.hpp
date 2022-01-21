#pragma once

#include "physics2D/rigidBody/CollisionsManifold.hpp"
#include "physics2D/primitives/Circle.hpp"
#include "physics2D/primitives/Collider2D.hpp"
#include "physics2D/primitives/Box2D.hpp"

// libs
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>

// std
#include <stdexcept>
#include <iostream>

/**
 * @brief box2D collisions by https://github.com/erincatto from https://github.com/erincatto/box2d-lite
 * 
 */

namespace physics2D::rigidBody::collisions{
    static CollisionsManifold findCollisionsFeatures(primitives::Circle a, primitives::Circle b){
        CollisionsManifold result;

        float sumRadii = a.getRadius() + b.getRadius();
        glm::vec2 distance = b.getCenter() - a.getCenter();

        if (glm::pow(distance.x, 2) + glm::pow(distance.y, 2) > glm::pow(sumRadii, 2)) return result;
        
        float depth = glm::abs(distance.length() - sumRadii) * 0.5f;
        glm::vec2 normal = glm::normalize(distance);
        float distanceToPoint = a.getRadius() - depth;
        glm::vec2 contactPoint = a.getCenter() + normal * distanceToPoint;

        result.setDeth(depth);
        result.setNormal(normal);
        result.getContactPoints().push_back(contactPoint);
        result.setColliding(true);

        return result;
    }

	inline glm::mat2 Abs(const glm::mat2& A){
		return glm::mat2(glm::abs(glm::column(A, 0)), glm::abs(glm::column(A, 1)));
	}

	enum Axis{
		FACE_A_X,
		FACE_A_Y,
		FACE_B_X,
		FACE_B_Y
	};

	union FeaturePair{
		struct Edges{
			char inEdge1;
			char outEdge1;
			char inEdge2;
			char outEdge2;
		};
		Edges e;
		int value;
	};

	enum EdgeNumbers{
		NO_EDGE = 0,
		EDGE1,
		EDGE2,
		EDGE3,
		EDGE4
	};

	struct ClipVertex{
		ClipVertex() { fp.value = 0; }
		glm::vec2 v;
		FeaturePair fp;
	};

	void Flip(FeaturePair& fp){
		std::swap(fp.e.inEdge1, fp.e.inEdge2);
		std::swap(fp.e.outEdge1, fp.e.outEdge2);
	}

	int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2], const glm::vec2& normal, float offset, char clipEdge){
		// Start with no output points
		int numOut = 0;

		// Calculate the distance of end points to the line
		float distance0 = glm::dot(normal, vIn[0].v) - offset;
		float distance1 = glm::dot(normal, vIn[1].v) - offset;

		// If the points are behind the plane
		if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
		if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f)
		{
			// Find intersection point of edge and plane
			float interp = distance0 / (distance0 - distance1);
			vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			if (distance0 > 0.0f)
			{
				vOut[numOut].fp = vIn[0].fp;
				vOut[numOut].fp.e.inEdge1 = clipEdge;
				vOut[numOut].fp.e.inEdge2 = NO_EDGE;
			}
			else
			{
				vOut[numOut].fp = vIn[1].fp;
				vOut[numOut].fp.e.outEdge1 = clipEdge;
				vOut[numOut].fp.e.outEdge2 = NO_EDGE;
			}
			++numOut;
		}

		return numOut;
	}

	static void ComputeIncidentEdge(ClipVertex c[2], const glm::vec2& h, const glm::vec2& pos, const glm::mat2& Rot, const glm::vec2& normal){
		// The normal is from the reference box. Convert it
		// to the incident boxe's frame and flip sign.
		glm::mat2 RotT = glm::transpose(Rot);
		glm::vec2 n = -(RotT * normal);
		glm::vec2 nAbs = glm::abs(n);

		if (nAbs.x > nAbs.y){
			if (glm::sign(n.x) > 0.0f){
				c[0].v = glm::vec2(h.x, -h.y);
				c[0].fp.e.inEdge2 = EDGE3;
				c[0].fp.e.outEdge2 = EDGE4;

				c[1].v = glm::vec2(h.x, h.y);
				c[1].fp.e.inEdge2 = EDGE4;
				c[1].fp.e.outEdge2 = EDGE1;
			}
			else{
				c[0].v = glm::vec2(-h.x, h.y);
				c[0].fp.e.inEdge2 = EDGE1;
				c[0].fp.e.outEdge2 = EDGE2;

				c[1].v = glm::vec2(-h.x, -h.y);
				c[1].fp.e.inEdge2 = EDGE2;
				c[1].fp.e.outEdge2 = EDGE3;
			}
		}
		else{
			if (glm::sign(n.y) > 0.0f){
				c[0].v = glm::vec2(h.x, h.y);
				c[0].fp.e.inEdge2 = EDGE4;
				c[0].fp.e.outEdge2 = EDGE1;

				c[1].v = glm::vec2(-h.x, h.y);
				c[1].fp.e.inEdge2 = EDGE1;
				c[1].fp.e.outEdge2 = EDGE2;
			}
			else{
				c[0].v = glm::vec2(-h.x, -h.y);
				c[0].fp.e.inEdge2 = EDGE2;
				c[0].fp.e.outEdge2 = EDGE3;

				c[1].v = glm::vec2(h.x, -h.y);
				c[1].fp.e.inEdge2 = EDGE3;
				c[1].fp.e.outEdge2 = EDGE4;
			}
		}

		c[0].v = pos + Rot * c[0].v;
		c[1].v = pos + Rot * c[1].v;
	}

	// The normal points from A to B
	static CollisionsManifold findCollisionsFeatures(primitives::Box2D &bodyA, primitives::Box2D &bodyB){
		// Setup
		glm::vec2 hA = 0.5f * glm::vec2(bodyA.getHalfSize().x);
		glm::vec2 hB = 0.5f * glm::vec2(bodyB.getHalfSize().x);

		glm::vec2 posA = bodyA.getPosition();
		glm::vec2 posB = bodyB.getPosition();

		glm::mat2 RotA(bodyA.getRotation()), RotB(bodyB.getRotation());

		glm::mat2 RotAT = glm::transpose(RotA);
		glm::mat2 RotBT = glm::transpose(RotB);

		glm::vec2 dp = posB - posA;
		glm::vec2 dA = RotAT * dp;
		glm::vec2 dB = RotBT * dp;

		glm::mat2 C = RotAT * RotB;
		glm::mat2 absC = Abs(C);
		glm::mat2 absCT = glm::transpose(absC);

		// Box A faces
		glm::vec2 faceA = glm::abs(dA) - hA - absC * hB;
		if (faceA.x > 0.0f || faceA.y > 0.0f)
			return CollisionsManifold();

		// Box B faces
		glm::vec2 faceB = glm::abs(dB) - absCT * hA - hB;
		if (faceB.x > 0.0f || faceB.y > 0.0f)
			return CollisionsManifold();

		// Find best axis
		Axis axis;
		float separation;
		glm::vec2 normal;

		// Box A faces
		axis = FACE_A_X;
		separation = faceA.x;
		normal = dA.x > 0.0f ? glm::column(RotA, 0) : -glm::column(RotA, 0);

		const float relativeTol = 0.95f;
		const float absoluteTol = 0.01f;

		if (faceA.y > relativeTol * separation + absoluteTol * hA.y){
			axis = FACE_A_Y;
			separation = faceA.y;
			normal = dA.y > 0.0f ? glm::column(RotA, 1) : -glm::column(RotA, 1);
		}

		// Box B faces
		if (faceB.x > relativeTol * separation + absoluteTol * hB.x){
			axis = FACE_B_X;
			separation = faceB.x;
			normal = dB.x > 0.0f ? glm::column(RotB, 0) : -glm::column(RotB, 0);
		}

		if (faceB.y > relativeTol * separation + absoluteTol * hB.y){
			axis = FACE_B_Y;
			separation = faceB.y;
			normal = dB.y > 0.0f ? glm::column(RotB, 1) : -glm::column(RotB, 1);
		}

		// Setup clipping plane data based on the separating axis
		glm::vec2 frontNormal, sideNormal;
		ClipVertex incidentEdge[2];
		float front, negSide, posSide;
		char negEdge, posEdge;

		// Compute the clipping lines and the line segment to be clipped.
		switch (axis){
			case FACE_A_X:
				{
					frontNormal = normal;
					front = glm::dot(posA, frontNormal) + hA.x;
					sideNormal = glm::column(RotA, 1);
					float side = glm::dot(posA, sideNormal);
					negSide = -side + hA.y;
					posSide =  side + hA.y;
					negEdge = EDGE3;
					posEdge = EDGE1;
					ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
				}
				break;

			case FACE_A_Y:
				{
					frontNormal = normal;
					front = glm::dot(posA, frontNormal) + hA.y;
					sideNormal = glm::column(RotA, 0);
					float side = glm::dot(posA, sideNormal);
					negSide = -side + hA.x;
					posSide =  side + hA.x;
					negEdge = EDGE2;
					posEdge = EDGE4;
					ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
				}
				break;

			case FACE_B_X:
				{
					frontNormal = -normal;
					front = glm::dot(posB, frontNormal) + hB.x;
					sideNormal = glm::column(RotB, 1);
					float side = glm::dot(posB, sideNormal);
					negSide = -side + hB.y;
					posSide =  side + hB.y;
					negEdge = EDGE3;
					posEdge = EDGE1;
					ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
				}
				break;

			case FACE_B_Y:
				{
					frontNormal = -normal;
					front = glm::dot(posB, frontNormal) + hB.y;
					sideNormal = glm::column(RotB, 0);
					float side = glm::dot(posB, sideNormal);
					negSide = -side + hB.x;
					posSide =  side + hB.x;
					negEdge = EDGE2;
					posEdge = EDGE4;
					ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
				}
				break;
		}

		// clip other face with 5 box planes (1 face plane, 4 edge planes)

		ClipVertex clipPoints1[2];
		ClipVertex clipPoints2[2];
		int np;

		// Clip to box side 1
		np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

		if (np < 2)
			return CollisionsManifold();

		// Clip to negative box side 1
		np = ClipSegmentToLine(clipPoints2, clipPoints1,  sideNormal, posSide, posEdge);

		if (np < 2)
			return CollisionsManifold();

		// Now clipPoints2 contains the clipping points.
		// Due to roundoff, it is possible that clipping removes all points.

        CollisionsManifold result;
        result.setDeth(separation);
        result.setNormal(normal);
        result.setColliding(true);

		int numContacts = 0;
		for (int i = 0; i < 2; ++i){
			float separation = glm::dot(frontNormal, clipPoints2[i].v) - front;

			if (separation <= 0){
				result.getContactPoints().push_back(clipPoints2[i].v - separation * frontNormal);
			}
		}

		return result;
	}

    static CollisionsManifold findCollisionsFeatures(primitives::Collider2D *a, primitives::Collider2D *b){
        if (primitives::Circle *c1 = dynamic_cast<primitives::Circle*>(a)){
			if (primitives::Circle *c2 = dynamic_cast<primitives::Circle*>(b))
				return findCollisionsFeatures(*c1, *c2);
        
        
        } else if (primitives::Box2D *b1 = dynamic_cast<primitives::Box2D*>(a)){
            if (primitives::Box2D *b2 = dynamic_cast<primitives::Box2D*>(b))
                return findCollisionsFeatures(*b1, *b2);

		} else {
			throw std::logic_error("unknown collider");
		}
		
		return CollisionsManifold();
    }
}