#pragma once

#include "physics2D/forces/ForceGenerator.hpp"

namespace physics2D::forces{
	class Gravity2D : public ForceGenerator{
		public:
			Gravity2D(glm::vec2 force = glm::vec2(0.f, 8.91f)) : ForceGenerator(), force{force} {}


			void updateForce(rigidBody::RigidBody &body, float dt) override{
				body.addForce(force * body.getMass());
			}

		private:
			glm::vec2 force;
	};
}