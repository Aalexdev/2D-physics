#pragma once

// libs
#include <glm/glm.hpp>

namespace physics2D::primitives{
	class RaycastResult{
		public:
			RaycastResult() {}

			void init(glm::vec2 point, glm::vec2 normal, float t, bool hit){
				this->point = point;
				this->normal = normal;
				this->t = t;
				this->hit = hit;
			}

			RaycastResult(glm::vec2 point, glm::vec2 normal, float t, bool hit){
				init(point, normal, t, hit);
			}

			/**
			 * @brief get the point
			 * @return glm::vec2 
			 */
			glm::vec2 getPoint() const noexcept {return point;}

			/**
			 * @brief get the normal of the point
			 * @return glm::vec2 
			 */
			glm::vec2 getNnormal() const noexcept {return normal;}

			static void reset(RaycastResult &result) noexcept{
				result.init(glm::vec2(0.f), glm::vec2(0.f), -1.f, false);
			}

			/**
			 * @brief reset the raycastResult
			 */
			void reset() noexcept {reset(*this);}

		private:
			glm::vec2 point = glm::vec2(0.f);
			glm::vec2 normal = glm::vec2(0.f);
			float t = -1.f;
			bool hit = false;

	};
}