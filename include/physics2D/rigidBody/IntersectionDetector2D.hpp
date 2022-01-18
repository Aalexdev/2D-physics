#pragma once

// libs
#include <glm/glm.hpp>

#include "Line2D.hpp"
#include "physics2D/primitives/Circle.hpp"
#include "physics2D/primitives/AABB.hpp"
#include "physics2D/primitives/Box2D.hpp"
#include "physics2D/constants.hpp"
#include "util/math.hpp"

namespace physics2D::rigidBody{
	namespace intersectionDetector{
		/**
		 * @brief get if the given point is on tyhe line or not
		 * 
		 * @param point the point to check
		 * @param line
		 */
		static inline bool pointOnLine(glm::vec2 point, Line2D line){
			glm::vec2 d = line.getEnd() - line.getStart();
			float m = d.x / d.y;

			float b = line.getEnd().y - (m * line.getEnd().x);
			return glm::epsilonEqual(point.y, m * point.x + b, precision::EPSILON);
		}

		/**
		 * @brief get if a point is inside a circle
		 * 
		 * @param point 
		 * @param circle 
		 */
		static inline bool pointInCircle(glm::vec2 point, primitives::Circle circle){
			glm::vec2 centerToPoint = point - circle.getCenter();
			return glm::pow(circle.getRadius(), 2.f) >= glm::pow(centerToPoint.x, 2) + glm::pow(centerToPoint.y, 2);
		}

		/**
		 * @brief get if the point is inside the AABB
		 * 
		 * @param point 
		 * @param box 
		 */
		static inline bool pointInAABB(glm::vec2 point, primitives::AABB box){
			glm::vec2 min = box.getMin();
			glm::vec2 max = box.getMax();

			if (point.x <= min.x || point.x >= max.x) return false;
			if (point.y <= min.y || point.y >= max.y) return false;
			return true;
		}

		/**
		 * @brief get if a point is inside the box
		 * 
		 * @param point 
		 * @param box 
		 * @return true 
		 * @return false 
		 */
		static inline bool pointInBox2D(glm::vec2 point, primitives::Box2D box){
			math::rotate2D(point, box.getRotation(), box.getPosition());

			glm::vec2 min = box.getMin();
			glm::vec2 max = box.getMax();

			if (point.x <= min.x || point.x >= max.x) return false;
			if (point.y <= min.y || point.y >= max.y) return false;
			return true;
		}
	}
}