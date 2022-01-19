#pragma once

// libs
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>

namespace math{
	/**
	 * @brief rotate the given vector around the origin
	 * 
	 * @param vector 
	 * @param angleDeg angle /!\ in degrees /!\ 
	 * @param origin 
	 */
	static inline void rotate2D(glm::vec2 &vector, float angleDeg, glm::vec2 origin){
		glm::vec2 moved = vector - origin;

		float cos = glm::cos(glm::radians(angleDeg));
		float sin = glm::sin(glm::radians(angleDeg));

		glm::vec2 prime = glm::vec2((moved.x * cos) - (moved.y * sin), (moved.x * sin) + (moved.y * cos));
		vector = prime + origin;
	}
}