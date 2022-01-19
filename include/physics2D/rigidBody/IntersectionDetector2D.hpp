#pragma once

// std
#include <limits>

// libs
#include <glm/glm.hpp>

#include "physics2D/primitives/Line2D.hpp"
#include "physics2D/primitives/Circle.hpp"
#include "physics2D/primitives/AABB.hpp"
#include "physics2D/primitives/Box2D.hpp"
#include "physics2D/primitives/Ray.hpp"
#include "physics2D/primitives/RaycastResult.hpp"
#include "physics2D/util/math.hpp"

namespace physics2D::rigidBody::intersectionDetector{
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
		return glm::epsilonEqual(point.y, m * point.x + b, std::numeric_limits<float>::min());
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
	 * @param circle 
	 * @param line 
	 * @return true 
	 * @return false 
	 */
	static inline bool CircleVsLine(primitives::Circle circle, primitives::Line2D line){
		return LineVsCircle(line, circle);
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

		primitives::Line2D line2D(start, end);
		primitives::AABB aabb(box.getMin(), box.getMax());

		return LineVsAABB(line2D, aabb);
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

	/**
	 * @brief 
	 * 
	 * @param box 
	 * @param ray 
	 * @param result 
	 * @return true 
	 * @return false 
	 */
	static inline bool rayCast(primitives::AABB box,  primitives::Ray ray, primitives::RaycastResult *result = nullptr){
		if (result) result->reset();
		glm::vec2 unitVec = ray.getDirection();

		unitVec.x = (unitVec.x != 0) ? 1.f / unitVec.x : 0.f;
		unitVec.y = (unitVec.y != 0) ? 1.f / unitVec.y : 0.f;

		glm::vec2 min = box.getMin() - ray.getOrigin() * unitVec;
		glm::vec2 max = box.getMax() - ray.getOrigin() * unitVec;

		float tMin = std::max(std::min(min.x, max.x), std::min(min.y, max.y));
		float tMax = std::min(std::max(min.x, max.x), std::max(min.y, max.y));

		if (tMax < 0.f || tMin > tMax) return false;

		float t = (tMin < 0.f) ? tMax : tMin;
		bool hit = t > 0.f;// && glm::pow(t, 2) < ray.getMaximum();

		if (!hit) return false;

		if (result){
			glm::vec2 point = ray.getOrigin() + ray.getDirection() * t;
			glm::vec2 normal = glm::normalize(ray.getOrigin() - point);

			result->init(point, normal, t, true);
		}
		return true;
	}

	/**
	 * @brief 
	 * 
	 * @param box 
	 * @param ray 
	 * @param result 
	 * @return true 
	 * @return false 
	 */
	static inline bool rayCast(primitives::Box2D box,  primitives::Ray ray, primitives::RaycastResult *result = nullptr){
		if (result) result->reset();
		
		glm::vec2 size = box.getHalfSize();
		glm::vec2 xAxis(1.f, 0.f);
		glm::vec2 yAxis(0.f, 1.f);

		math::rotate2D(xAxis, -box.getRotation(), glm::vec2(0.f));
		math::rotate2D(yAxis, -box.getRotation(), glm::vec2(0.f));

		glm::vec2 p = box.getPosition() - ray.getOrigin();
		glm::vec2 f = glm::vec2(glm::dot(xAxis, ray.getDirection()), glm::dot(yAxis, ray.getDirection()));

		glm::vec2 e = glm::vec2(glm::dot(xAxis, p), glm::dot(yAxis, p));

		float tArr[] = {0, 0, 0, 0};
		for (int i=0; i<2; i++){

			if (glm::epsilonEqual(f[i], 0.f, std::numeric_limits<float>::min())){
				if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0) return false;
			
				f[i] = std::numeric_limits<float>::min();
			}
			tArr[i*2] =   e[i] + size[i] / f[i];
			tArr[i*2+1] = e[i] - size[i] / f[i];
		}

		float tMin = std::max(std::min(tArr[0], tArr[1]), std::min(tArr[2], tArr[3]));
		float tMax = std::min(std::max(tArr[0], tArr[1]), std::max(tArr[2], tArr[3]));

		float t = (tMin < 0.f) ? tMax : tMin;
		bool hit = t > 0.f;// && glm::pow(t, 2) < ray.getMaximum();

		if (!hit) return false;

		if (result){
			glm::vec2 point = ray.getOrigin() + ray.getDirection() * t;
			glm::vec2 normal = glm::normalize(ray.getOrigin() - point);

			result->init(point, normal, t, true);
		}

		return true;
	}
	
	/**
	 * @brief 
	 * 
	 * @param c1 
	 * @param c2 
	 * @return true 
	 * @return false 
	 */
	static inline bool CircleVsCircle(primitives::Circle c1, primitives::Circle c2){
		glm::vec2 betweenCenters = c1.getCenter() - c2.getCenter();
		float radiiSum = c1.getRadius() + c2.getRadius();
		return glm::pow(betweenCenters.x, 2) + glm::pow(betweenCenters.y, 2) <= glm::pow(radiiSum, 2); 
	}

	/**
	 * @brief 
	 * 
	 * @param aabb 
	 * @param circle 
	 * @return true 
	 * @return false 
	 */
	static inline bool AABBVsCircle(primitives::AABB box, primitives::Circle circle){
		glm::vec2 min = box.getMin();
		glm::vec2 max = box.getMax();

		glm::vec2 closestPointToCircle = circle.getCenter();

		if (closestPointToCircle.x < min.x){
			closestPointToCircle.x = min.x;
		} else if (closestPointToCircle.x > max.x){
			closestPointToCircle.x = max.x;
		}

		if (closestPointToCircle.y < min.y){
			closestPointToCircle.y = min.y;
		} else if (closestPointToCircle.y > max.y){
			closestPointToCircle.y = max.y;
		}

		glm::vec2 circleToBox = circle.getCenter() - closestPointToCircle;
		return glm::pow(circleToBox.x, 2) + glm::pow(circleToBox.y, 2) < glm::pow(circle.getRadius(), 2);
	}

	/**
	 * @brief 
	 * 
	 * @param circle 
	 * @param aabb 
	 * @return true 
	 * @return false 
	 */
	static inline bool CircleVsAABB(primitives::Circle circle, primitives::AABB box){
		return AABBVsCircle(box, circle);
	}

	/**
	 * @brief 
	 * 
	 * @param box 
	 * @param circle 
	 * @return true 
	 * @return false 
	 */
	static inline bool Box2DVsCircle(primitives::Box2D box, primitives::Circle circle){
		glm::vec2 min = glm::vec2(0.f);
		glm::vec2 max = box.getSize();

		glm::vec2 r = circle.getCenter() - box.getPosition();
		math::rotate2D(r, -box.getRotation(), glm::vec2(0.f));

		glm::vec localCirclePos = r + box.getHalfSize();
		glm::vec2 closestPointToCircle = localCirclePos;

		if (closestPointToCircle.x < min.x){
			closestPointToCircle.x = min.x;
		} else if (closestPointToCircle.x > max.x){
			closestPointToCircle.x = max.x;
		}

		if (closestPointToCircle.y < min.y){
			closestPointToCircle.y = min.y;
		} else if (closestPointToCircle.y > max.y){
			closestPointToCircle.y = max.y;
		}

		glm::vec2 circleToBox = localCirclePos - closestPointToCircle;
		return glm::pow(circleToBox.x, 2) + glm::pow(circleToBox.y, 2) < glm::pow(circle.getRadius(), 2);
	}
	
	/**
	 * @brief 
	 * 
	 * @param circle 
	 * @param box 
	 * @return true 
	 * @return false 
	 */
	static inline bool CircleVsBox2d(primitives::Circle circle, primitives::Box2D box){
		return Box2DVsCircle(box, circle);
	}

	/**
	 * @brief Get the Interval object
	 * 
	 * @param rectangle 
	 * @param axis 
	 * @return glm::vec2 
	 */
	static inline glm::vec2 getInterval(primitives::AABB rectangle, glm::vec2 axis){
		glm::vec2 result = glm::vec2(0.f);

		glm::vec2 min = rectangle.getMin();
		glm::vec2 max = rectangle.getMax();

		glm::vec2 vertices[] = {
			glm::vec2(min),
			glm::vec2(min.x, max.y),
			glm::vec2(max),
			glm::vec2(max.x, min.y)
		};

		result.x = glm::dot(axis, vertices[0]);
		result.y = result.x;

		for (int i=0; i<4; i++){
			float projection = glm::dot(axis, vertices[i]);

			if (projection < result.x){
				result.x = projection;
			}

			if (projection > result.y){
				result.y = projection;
			}
		}

		return result;
	}

	/**
	 * @brief Get the Interval object
	 * 
	 * @param rectangle 
	 * @param axis 
	 * @return glm::vec2 
	 */
	static inline glm::vec2 getInterval(primitives::Box2D rectangle, glm::vec2 axis){
		glm::vec2 result = glm::vec2(0.f);

		glm::vec2 min = rectangle.getMin();
		glm::vec2 max = rectangle.getMax();

		std::vector<glm::vec2> vertices = rectangle.getVertices();

		result.x = glm::dot(axis, vertices[0]);
		result.y = result.x;

		for (int i=0; i<4; i++){
			float projection = glm::dot(axis, vertices[i]);

			if (projection < result.x){
				result.x = projection;
			}

			if (projection > result.y){
				result.y = projection;
			}
		}

		return result;
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool overlapOnAxis(primitives::AABB b1, primitives::AABB b2, glm::vec2 axis){
		glm::vec2 interval1 = getInterval(b1, axis);
		glm::vec2 interval2 = getInterval(b2, axis);

		return ((interval2.x <= interval1.y) && (interval1.x <= interval2.y));
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool overlapOnAxis(primitives::AABB b1, primitives::Box2D b2, glm::vec2 axis){
		glm::vec2 interval1 = getInterval(b1, axis);
		glm::vec2 interval2 = getInterval(b2, axis);

		return ((interval2.x <= interval1.y) && (interval1.x <= interval2.y));
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool overlapOnAxis(primitives::Box2D b1, primitives::Box2D b2, glm::vec2 axis){
		glm::vec2 interval1 = getInterval(b1, axis);
		glm::vec2 interval2 = getInterval(b2, axis);

		return ((interval2.x <= interval1.y) && (interval1.x <= interval2.y));
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool AABBVsAABB(primitives::AABB b1, primitives::AABB b2){
		glm::vec2 axesToTest[] = {glm::vec2(0.f, 1.f), glm::vec2(1.f, 0.f)};

		for (int i=0; i<2; i++){
			if (!overlapOnAxis(b1, b2, axesToTest[i])) return false;
		}

		return true;
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool AABBVsBox2D(primitives::AABB b1, primitives::Box2D b2){
		glm::vec2 axesToTest[] = {
			glm::vec2(0.f, 1.f),
			glm::vec2(1.f, 0.f),
			glm::vec2(0.f, 1.f),
			glm::vec2(1.f, 0.f)
		};

		math::rotate2D(axesToTest[2], b2.getRotation(), glm::vec2(0.f));
		math::rotate2D(axesToTest[3], b2.getRotation(), glm::vec2(0.f));

		for (int i=0; i<4; i++){
			if (!overlapOnAxis(b1, b2, axesToTest[i])) return false;
		}

		return true;
	}

	/**
	 * @brief 
	 * 
	 * @param b1 
	 * @param b2 
	 * @return true 
	 * @return false 
	 */
	static inline bool Box2dDVsAABB(primitives::Box2D b1, primitives::AABB b2){
		return AABBVsBox2D(b2, b1);
	}

}