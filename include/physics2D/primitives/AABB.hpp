#pragma once

/**
 * @brief AABB stand for Axis Aligned Bounding Box, which is just a rectangle who never rotate
 * 
 */

// libs
#include <glm/glm.hpp>

#include "physics2D/rigidBody/RigidBody.hpp"

namespace physics2D::primitives{
	class AABB{
		public:
			AABB(){}

			AABB(glm::vec2 min, glm::vec2 max){
				if (glm::dot(min, min) > glm::dot(max, max)) std::swap(min, max);

				size = max - min;
				halfSize = size / 2.f;
			}

			/**
			 * @brief get the size of the AABB object
			 * @return glm::vec2 
			 */
			glm::vec2 getSize() const noexcept {return size;}

			/**
			 * @brief get the minimal coordonates of the AABB object
			 * @return glm::vec2 
			 */
			glm::vec2 getMin() const noexcept {return rigidBody.getPosition() - halfSize;}

			/**
			 * @brief get the maximal coordonates of the AABB object
			 * @return glm::vec2 
			 */
			glm::vec2 getMax() const noexcept {return rigidBody.getPosition() + halfSize;}

		private:
			glm::vec2 size = glm::vec2(0.f);
			glm::vec2 halfSize;
			rigidBody::RigidBody rigidBody;
	};
}