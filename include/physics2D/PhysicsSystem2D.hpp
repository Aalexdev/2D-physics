#pragma once

#include "physics2D/forces/ForceRegistery.hpp"
#include "physics2D/rigidBody/RigidBody.hpp"
#include "physics2D/forces/Gravity2D.hpp"

// libs
#include <glm/glm.hpp>

// std
#include <list>
#include <memory>

namespace physics2D{
	class PhysicsSystem{
		public:
			PhysicsSystem(float fixedUpdateDt) : fixedUpdateDt{fixedUpdateDt}{

			}

			void update(float dt){
				fixedUpdate();
			}

			void fixedUpdate(){
				forceRegistery.updateForces(fixedUpdateDt);

				for (rigidBody::RigidBody& body : rigidBodies){
					body.physicsUpdate(fixedUpdateDt);
				}
			}

			void addRigidBody(rigidBody::RigidBody body){
				rigidBodies.push_back(body);
				forceRegistery.add(rigidBodies.back(), gravity);
			}

		private:
			forces::ForceRegistery forceRegistery;
			std::list<rigidBody::RigidBody> rigidBodies;

			forces::Gravity2D gravity;
			float fixedUpdateDt;
	};
}