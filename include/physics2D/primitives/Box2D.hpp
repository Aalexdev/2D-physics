#pragma once

// libs
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>

// std
#include <vector>
#include <limits>

#include "physics2D/rigidBody/RigidBody.hpp"
#include "physics2D/util/math.hpp"

namespace physics2D::primitives{
	class Box2D{
		public:
			Box2D(){}

			Box2D(glm::vec2 min, glm::vec2 max){
				if (glm::dot(min, min) > glm::dot(max, max)) std::swap(min, max);

				size = max - min;
				halfSize = size / 2.f;
			}

			/**
			 * @brief get the minimal coordonates of the Box2D object
			 * @return glm::vec2 
			 */
			glm::vec2 getMin() const noexcept {return rigidBody.getPosition() - halfSize;}

			/**
			 * @brief get the maximal coordonates of the Box2D object
			 * @return glm::vec2 
			 */
			glm::vec2 getMax() const noexcept {return rigidBody.getPosition() + halfSize;}

			/**
			 * @brief get the vertices of the box
			 * @return std::vector<glm::vec2> 
			 */
			std::vector<glm::vec2> getVertices() const{
				std::vector<glm::vec2> vertices(4);

				glm::vec2 min = getMin();
				glm::vec2 max = getMax();

				vertices[0] = min;
				vertices[1] = {min.x, max.y};
				vertices[2] = {max.x, min.y};
				vertices[3] = max;

				if (glm::epsilonNotEqual(rigidBody.getRotation(), 0.f, std::numeric_limits<float>::min())){
					for (glm::vec2 &vert : vertices){
						math::rotate2D(vert, rigidBody.getRotation(), rigidBody.getPosition());
					}
				}

				return vertices;
			}

			/**
			 * @brief get the rotation of the box
			 * @return float 
			 */
			float getRotation() const noexcept {return rigidBody.getRotation();}

			/**
			 * @brief get the position of the box
			 * @return glm::vec2 
			 */
			glm::vec2 getPosition() const noexcept {return rigidBody.getPosition();}

			/**
			 * @brief get the size of the box
			 * @return glm::vec2 
			 */
			glm::vec2 getSize() const noexcept {return size;}

			/**
			 * @brief get the size of the box divided by two
			 * @return glm::vec2 
			 */
			glm::vec2 getHalfSize() const noexcept {return halfSize;}

		private:
			glm::vec2 size = glm::vec2(0.f);
			glm::vec2 halfSize;
			rigidBody::RigidBody rigidBody;
	};
}