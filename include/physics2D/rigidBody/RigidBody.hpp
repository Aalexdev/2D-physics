#pragma once

#include "components/Transform.hpp"
#include "physics2D/primitives/Collider2D.hpp"

// libs
#include <glm/glm.hpp>

// std
#include <iostream>

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
			
			/**
			 * @brief set the transform position
			 * @param position 
			 */
			void setTransform(glm::vec2 position){
				this->position = position;
			}

			/**
			 * @brief set the transform rotation
			 * @param rotation 
			 */
			void setTransform(float rotation){
				this->rotation = rotation;
			}

			/**
			 * @brief get the mass of the body (in Kg)
			 * @return float 
			 */
			float getMass() const noexcept {return mass;}

			/**
			 * @brief set the mass of the body (in Kg)
			 * @param mass 
			 */
			void setMass(float mass) {
				this->mass = mass;

				if (mass != 0.0f){
					invertMass = 1.f / mass;
				}
			}

			/**
			 * @brief update the body
			 * @param dt the delta time in secondes
			 */
			void physicsUpdate(float dt){
				if (mass == 0.f) return;

				glm::vec2 acceleration = forces * invertMass;
				linearVelocity += acceleration * dt;

				position += linearVelocity * dt;

				syncCollisionsTransforms();
				forces = glm::vec2(0.f);
			}

			/**
			 * @brief syncronise the transform component (if set)
			 */
			void syncCollisionsTransforms(){
				if (transform){
					transform->setPosition(position);
					transform->setRotation(rotation);
				}
			}

			/**
			 * @brief set the transform component
			 * @param t 
			 */
			void setTransform(components::Transform *t){
				transform = t;
			}

			/**
			 * @brief apply a force onto the body
			 * @param force 
			 */
			void addForce(glm::vec2 force){
				forces += force;
			}

			/**
			 * @brief get if the baody has an infinit mass
			 * 
			 * @return true 
			 * @return false 
			 */
			bool hasInfinitMass() const noexcept {return mass == 0.f;}

			/**
			 * @brief set the body collider
			 * @param collider 
			 */
			void setCollider(primitives::Collider2D *collider) {this->collider = collider;}

			/**
			 * @brief get the body collider
			 * 
			 * @return primitives::Collider2D* 
			 */
			primitives::Collider2D *getCollider() const noexcept {return collider;}

			/**
			 * @brief get 1 over the mass (1.0 / masss)
			 * @return float 
			 */
			float getInvertMass() const noexcept {return invertMass;}

			/**
			 * @brief get the velocity of the object
			 * @return glm::vec2 
			 */
			glm::vec2 getVelocity() const noexcept {return linearVelocity;}

			/**
			 * @brief set the velocity of the body
			 * @param newVelocity 
			 */
			void setVelocity(glm::vec2 newVelocity) noexcept {linearVelocity = newVelocity;}

			/**
			 * @brief get the coeffitient of restitution
			 * @return float 
			 */
			float cor() const noexcept {return coeffitientOfRestitution;}

			/**
			 * @brief set the coeffitient of restitution
			 * @param coef 
			 */
			void setCor(float coef) noexcept {coeffitientOfRestitution = coef;}

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
			float coeffitientOfRestitution = 1.f;

			bool fixedRotation = false;
			components::Transform *transform = nullptr;
			primitives::Collider2D *collider = nullptr;
	};
}