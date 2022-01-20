#pragma once

#include "Transform.hpp"

// libs
#include <glm/glm.hpp>

/**
 * @brief could be used as a component in a entity components system
 * 
 */
namespace physics2D::rigidBody{
	class RigidBody{
		public:
			RigidBody(){}

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

			float getMass() const noexcept {return mass;}

			void setMass(float mass) {
				this->mass = mass;

				if (mass != 0.0f){
					invertMass = 1.f / mass;
				}
			}

			void physicsUpdate(float dt){
				if (mass == 0.f) return;

				glm::vec2 acceleration = forces * invertMass;
				linearVelocity += acceleration * dt;

				position += linearVelocity * dt;

				syncCollisionsTransforms();
				forces = glm::vec2(0.f);
			}

			void syncCollisionsTransforms(){
				if (transform){
					transform->position = position;
					transform->rotation = rotation;
				}
			}

			void setTransform(Transform *t){
				transform = t;
			}

			void addForce(glm::vec2 force){
				forces += force;
			}

		private:
			glm::vec2 position = glm::vec2(0.f);
			glm::vec2 linearVelocity = glm::vec2(0.f);
			glm::vec2 forces = glm::vec2(0.f);

			float angularVelocity = 0.f;
			float linearDamping = 0.f;
			float angularDamping = 0.f;
			float rotation = 0.f;
			float mass = 0.f;
			float invertMass = 0.f;

			bool fixedRotation = false;
			Transform *transform = nullptr;
	};
}