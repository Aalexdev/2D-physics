#pragma once

// libs
#include <glm/glm.hpp>

namespace physics{
	class Body{
		public:

			Body();
			~Body();

			/**
			 * @brief get the position of the body
			 * @return glm::vec2 
			 */
			glm::vec2 getPosition() const noexcept {return position;}

			/**
			 * @brief get the velocity of the body
			 * @return glm::vec2 
			 */
			glm::vec2 getVelocity() const noexcept {return velocity;}

			/**
			 * @brief get the mass of the body
			 * @return float 
			 */
			float getMass() const noexcept {return mass;}

			/**
			 * @brief update the physic body from the delta time
			 * @param dt delta time
			 */
			virtual void update(const float &dt);

			/**
			 * @brief apply the given force to the body forces
			 * @param force 
			 */
			void applyForce(glm::vec2 force) {forces += force;}
		
		protected:
			glm::vec2 position;
			glm::vec2 velocity;
			glm::vec2 forces;
			float mass;
	};
}