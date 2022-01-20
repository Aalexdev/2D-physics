#pragma once

#include "physics2D/rigidBody/RigidBody.hpp"

namespace physics2D::forces{
	class ForceGenerator{
		public:
			ForceGenerator(){};

			virtual void updateForce(rigidBody::RigidBody &body, float dt) {}
	};
}