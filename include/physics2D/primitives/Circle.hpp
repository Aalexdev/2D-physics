#pragma once

#include "physics2D/rigidBody/RigidBody.hpp"
#include "physics2D/primitives/Collider2D.hpp"

namespace physics2D::primitives{
	class Circle : public Collider2D{
		public:
			Circle() : radius{1}, Collider2D() {}
			Circle(float radius) : radius{radius}, Collider2D() {}

			/**
			 * @brief set the radius of the circle
			 * @param newRadius 
			 */
			void setRadius(const float newRadius) noexcept {radius = newRadius;}

			/**
			 * @brief get the radius of the circle
			 * @return float 
			 */
			float getRadius() const noexcept {return radius;}

			/**
			 * @brief get the center of the circle
			 * @return glm::vec2 
			 */
			glm::vec2 getCenter() const noexcept {return rigidBody->getPosition();}
			void setRigidBody(rigidBody::RigidBody *rigidBody) noexcept {
				this->rigidBody = rigidBody;
			}
		
		private:
			float radius;
			rigidBody::RigidBody *rigidBody;
	};
}