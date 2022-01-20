#pragma once

#include "physics2D/rigidBody/CollisionsManifold.hpp"
#include "physics2D/primitives/Circle.hpp"
#include "physics2D/primitives/Collider2D.hpp"

// libs
#include <glm/glm.hpp>

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

    static CollisionsManifold findCollisionsFeatures(priitives::Collider a, primitives::Collider2D b){
        
    }
}