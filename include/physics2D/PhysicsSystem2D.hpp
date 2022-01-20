#pragma once

#include "physics2D/forces/ForceRegistery.hpp"
#include "physics2D/rigidBody/RigidBody.hpp"
#include "physics2D/forces/Gravity2D.hpp"
#include "physics2D/rigidBody/CollisionsManifold.hpp"
#include "physics2D/rigidBody/Collisions.hpp"
#include "physics2D/primitives/Collider2D.hpp"
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
				bodies1.clear();
				bodies2.clear();
				collisions.clear();

				for (auto i : mEntities){
					for (auto j : mEntities){
						if (i == j) continue;

						rigidBody::CollisionsManifold result;
						rigidBody::RigidBody &r1 = coordinator.GetComponent<rigidBody::RigidBody>(i);
						rigidBody::RigidBody &r2 = coordinator.GetComponent<rigidBody::RigidBody>(j);
						primitives::Collider2D *c1 = r1.getCollider();
						primitives::Collider2D *c2 = r2.getCollider();

						if (c1 != nullptr && c2 != nullptr && !(r1.hasInfinitMass() && r2.hasInfinitMass())){
							result = rigidBody::collisions::findCollisionsFeatures(*c1, *c2);
						}
						
					}
				}

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
			forces::ForceRegistery forceRegistery;
			forces::Gravity2D gravity;

			float fixedUpdateDt;

			std::list<rigidBody::RigidBody*> bodies1;
			std::list<rigidBody::RigidBody*> bodies2;
			std::list<rigidBody::CollisionsManifold> collisions;
	};
}