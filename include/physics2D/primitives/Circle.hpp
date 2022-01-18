#pragma once

#include "physics2D/rigidBody/RigidBody.hpp"

namespace physics2D::primitives{
	class Circle{
		public:
			Circle() : radius{1} {}
			Circle(float radius) : radius{radius} {}

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
			glm::vec2 getCenter() const noexcept {return rigidBody.getPosition();}
		
		private:
			float radius;
			rigidBody::RigidBody rigidBody;
	};
}