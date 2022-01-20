#pragma once

#include "physics2D/forces/ForceRegistery.hpp"
#include "physics2D/rigidBody/RigidBody.hpp"
#include "physics2D/forces/Gravity2D.hpp"
#include "ECS.hpp"

// libs
#include <glm/glm.hpp>

// std
#include <list>
#include <memory>

namespace physics2D{
	class PhysicsSystem : public ECS::System{
		public:
			PhysicsSystem(float fixedUpdateDt, ECS::Coordinator &coordinator) : fixedUpdateDt{fixedUpdateDt}, coordinator{coordinator}{

			}

			void update(float dt){
				fixedUpdate();
			}

			void fixedUpdate(){
				forceRegistery.updateForces(fixedUpdateDt);

				for (auto entity : mEntities){
					coordinator.GetComponent<rigidBody::RigidBody>(entity).physicsUpdate(fixedUpdateDt);
				}
			}

			void addEntity(ECS::Entity entity){
				forceRegistery.add(coordinator.GetComponent<rigidBody::RigidBody>(entity), gravity);
				coordinator.GetComponent<rigidBody::RigidBody>(entity).setTransform(&coordinator.GetComponent<components::Transform>(entity));
			}

		private:
			ECS::Coordinator &coordinator;
			float fixedUpdateDt;

			forces::ForceRegistery forceRegistery;
			forces::Gravity2D gravity;
	};
}