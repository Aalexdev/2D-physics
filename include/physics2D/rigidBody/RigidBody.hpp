#pragma once

// libs
#include <glm/glm.hpp>

/**
 * @brief could be used as a component in a entity components system
 * 
 */
namespace physics2D::rigidBody{
	class RigidBody{
		public:
			RigidBody();

			/**
			 * @brief set the position of the rigidBody
			 * @param newPos 
			 */
			void setPosition(glm::vec2 newPos) noexcept {position = newPos;}

			/**
			 * @brief get the position of the rigidBody
			 * @return glm::vec2 
			 */
			glm::vec2 getPosition() const noexcept {return position;}

			/**
			 * @brief Get the rotation of the rigidBody
			 * @return float 
			 */
			float getRotation() const noexcept {return rotation;}

			/**
			 * @brief set the rotation of the rigidBody
			 * @param newRotation 
			 */
			void setRotation(float newRotation) {rotation = newRotation;}

		private:
			glm::vec2 position = glm::vec2(0.f);
			float rotation = 0.f;
	};
}