#pragma once

// libs
#include <glm/glm.hpp>

#include "physics2D/primitives/Line2D.hpp"
#include "physics2D/primitives/Circle.hpp"
#include "physics2D/primitives/AABB.hpp"
#include "physics2D/primitives/Box2D.hpp"
#include "physics2D/primitives/Ray.hpp"
#include "physics2D/primitives/RaycastResult.hpp"
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
		static inline bool pointOnLine(glm::vec2 point, primitives::Line2D line){
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
			math::rotate2D(point, -box.getRotation(), box.getPosition());

			glm::vec2 min = box.getMin();
			glm::vec2 max = box.getMax();

			if (point.x <= min.x || point.x >= max.x) return false;
			if (point.y <= min.y || point.y >= max.y) return false;
			return true;
		}

		/**
		 * @brief get if the line overlapping the circle
		 * 
		 * @param line 
		 * @param circle 
		 * @return true 
		 * @return false 
		 */
		static inline bool LineVsCircle(primitives::Line2D line, primitives::Circle circle){
			if (pointInCircle(line.getStart(), circle) || pointInCircle(line.getEnd(), circle)) return true;

			glm::vec2 ab = line.getEnd() - line.getStart();
			glm::vec2 centerToStart = circle.getCenter() - line.getStart();
			float t = glm::dot(centerToStart, ab) / glm::dot(ab, ab);

			if (t < 0.0f && t > 1.f) return false;

			glm::vec2 closestPoint = line.getStart() + ab * t;
			return pointInCircle(closestPoint, circle);
		}

		/**
		 * @brief 
		 * 
		 * @param line 
		 * @param box 
		 * @return true 
		 * @return false 
		 */
		static inline bool LineVsAABB(primitives::Line2D line, primitives::AABB box){
			if (pointInAABB(line.getStart(), box) || pointInAABB(line.getEnd(), box)) return true;

			glm::vec2 unitVec = glm::normalize(line.getEnd() - line.getStart());

			unitVec.x = (unitVec.x != 0) ? 1.f / unitVec.x : 0.f;
			unitVec.y = (unitVec.y != 0) ? 1.f / unitVec.y : 0.f;

			glm::vec2 min = box.getMin() - line.getStart() * unitVec;
			glm::vec2 max = box.getMax() - line.getStart() * unitVec;

			float tMin = std::max(std::min(min.x, max.x), std::min(min.y, max.y));
			float tMax = std::min(std::max(min.x, max.x), std::max(min.y, max.y));

			if (tMax < 0.f || tMin > tMax) return false;

			float t = (tMin < 0.f) ? tMax : tMin;
			return t > 0 && glm::pow(t, 2) < line.lenghtSquared();
		}

		/**
		 * @brief 
		 * 
		 * @param line 
		 * @param box 
		 * @return true 
		 * @return false 
		 */
		static inline bool LineVsBox2D(primitives::Line2D line, primitives::Box2D box){
			glm::vec2 start = line.getStart();
			glm::vec2 end = line.getEnd();

			math::rotate2D(start, -box.getRotation(), box.getPosition());
			math::rotate2D(end, -box.getRotation(), box.getPosition());

			primitives::Line2D line(start, end);
			primitives::AABB aabb(box.getMin(), box.getMax());

			return LineVsAABB(line, aabb);
		}

		/**
		 * @brief 
		 * 
		 * @param circle 
		 * @param ray 
		 * @param result 
		 * @return true 
		 * @return false 
		 */
		static inline bool rayCast(primitives::Circle circle, primitives::Ray ray, primitives::RaycastResult *result = nullptr){
			if (result) primitives::RaycastResult::reset(*result);

			glm::vec2 originToCircle = circle.getCenter() - ray.getOrigin();
			float radiusSquared = glm::pow(circle.getRadius(), 2);
			float originToCircleLenghtSquared = glm::pow(originToCircle.x, 2) + glm::pow(originToCircle.y, 2);

			float a = glm::dot(originToCircle, ray.getDirection());
			float bSq = originToCircleLenghtSquared - glm::pow(a, 2);

			if (radiusSquared - bSq < 0.f) return false;

			float f = glm::sqrt(radiusSquared - bSq);
			float t = 0;

			if (originToCircleLenghtSquared < radiusSquared){
				t = a + f;
			} else {
				t = a - f;
			}

			if (result){
				glm::vec2 point = ray.getOrigin() + ray.getDirection() * t;
				glm::vec2 normal = glm::normalize(point - circle.getCenter());

				result->init(point, normal, t, true);
			}

			return true;
		}
	}
}