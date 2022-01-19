#pragma once

#include <glm/glm.hpp>

namespace physics2D::primitives{
	class Ray{
		public:
			Ray();
			Ray(glm::vec2 origin, glm::vec2 direction) : origin{origin}, direction{glm::normalize(direction)} {}

			/**
			 * @brief get the origin of the ray, the starting point
			 * @return glm::vec2 
			 */
			glm::vec2 getOrigin() const noexcept {return origin;}

			/**
			 * @brief get the direction where the ray facing
			 * @return glm::vec2 
			 */
			glm::vec2 getDirection() const noexcept {return direction;}

		private:
			glm::vec2 origin = glm::vec2(0.f);
			glm::vec2 direction = glm::vec2(0.f);
	};
}