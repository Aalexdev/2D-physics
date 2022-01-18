#include "physics/PhysicBody.hpp"
#include "physics/constants.hpp"

namespace physics{
	void Body::update(const float &dt){
		velocity += forces;
		position += velocity * dt;

		// reset forces
		forces = glm::vec2(0.f);
	}
}