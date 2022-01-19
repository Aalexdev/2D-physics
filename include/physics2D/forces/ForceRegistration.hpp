#pragma once

#include "physics2D/forces/ForceGenerator.hpp"
#include "physics2D/rigidBody/RigidBody.hpp"

namespace physics2D::forces{
	class ForceRegistration{
		public:
			ForceRegistration(ForceGenerator &fg, rigidBody::RigidBody &body) : fg{fg}, body{body}{}

			bool operator==(ForceRegistration &other) const noexcept {
				return &other.body == &this->body && &other.fg == &this->fg;
			}

			ForceGenerator &getForceGenerator() const noexcept {return fg;}
			rigidBody::RigidBody &getBody() const noexcept {return body;}

		private:
			ForceGenerator &fg;
			rigidBody::RigidBody &body;
	};
}