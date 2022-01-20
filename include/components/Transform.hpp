#pragma once

// libs
#include <glm/glm.hpp>

namespace components{
	class Transform{
		public:
			Transform(){}

			/**
			 * @brief get the position of the transform component
			 * @return glm::vec2 
			 */
			glm::vec2 getPosition() const noexcept {return position;}

			/**
			 * @brief set the position of the transform
			 * @param newPosition 
			 */
			void setPosition(glm::vec2 newPosition) {position = newPosition;}

			/**
			 * @brief get the rotation of the transform
			 * @return float 
			 */
			float getRotation() const noexcept {return rotation;}

			/**
			 * @brief set the rotation of the transform
			 * @param newRotation 
			 */
			void setRotation(float newRotation) {rotation = newRotation;}
		
		private:
			glm::vec2 position = glm::vec2(0.f);
			float rotation = 0;
	};
}