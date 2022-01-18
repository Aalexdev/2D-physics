#pragma once

// libs
#include <glm/glm.hpp>

/**
 * @brief could be used as a component in a entity components system
 * 
 */
namespace physics2D::primitives{
	class Collider2D{
		public:
			Collider2D(){}

			// TODO : implement
			// float getInertiaTensor(float mass) const noexcept;

		protected:
			glm::vec2 offset = glm::vec2(0.f);
	};
}