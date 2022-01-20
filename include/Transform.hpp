#pragma once

// libs
#include <glm/glm.hpp>

class Transform{
	public:
		Transform(){}

		glm::vec2 position = glm::vec2(0.f);
		float rotation = 0;
};