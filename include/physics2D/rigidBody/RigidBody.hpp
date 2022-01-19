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
			 * @brief set the rigidBody attributes
			 * 
			 * @param position 
			 * @param angle 
			 */
			void setTransform(glm::vec2 position, float rotation){
				this->position = position;
				this->rotation = rotation;
			}
			
			void setTransform(glm::vec2 position){
				this->position = position;
			}

			void setTransform(float rotation){
				this->rotation = rotation;
			}

		private:
			glm::vec2 position = glm::vec2(0.f);
			glm::vec2 linearVelocity = glm::vec2(0.f);

			float angularVelocity = 0.f;
			float linearDamping = 0.f;
			float angularDamping = 0.f;
			float rotation = 0.f;

			bool fixedRotation = false;
	};
}