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
#include <vector>
#include <memory>

namespace physics2D{
	class PhysicsSystem : public ECS::System{
		public:
			PhysicsSystem(float fixedUpdateDt, ECS::Coordinator &coordinator) : fixedUpdateDt{fixedUpdateDt}, coordinator{coordinator}{
				bodies1.resize(collisionsPoolSize);
				bodies2.resize(collisionsPoolSize);
				collisions.resize(collisionsPoolSize);
			}

			void update(float dt){
				fixedUpdate();
			}

			void fixedUpdate(){
				collisionsCount = 0;

				for (auto i : mEntities){
					for (auto j : mEntities){
						if (i == j) continue;

						rigidBody::CollisionsManifold result;
						rigidBody::RigidBody &r1 = coordinator.GetComponent<rigidBody::RigidBody>(i);
						rigidBody::RigidBody &r2 = coordinator.GetComponent<rigidBody::RigidBody>(j);
						primitives::Collider2D *c1 = r1.getCollider();
						primitives::Collider2D *c2 = r2.getCollider();

						if (c1 != nullptr && c2 != nullptr && !(r1.hasInfinitMass() && r2.hasInfinitMass())){
							result = rigidBody::collisions::findCollisionsFeatures(c1, c2);
						}
						
						if (result.isColliding()){
							if (collisionsCount > collisionsPoolSize){
								collisionsPoolSize ++;

								bodies1.push_back(&r1);
								bodies2.push_back(&r2);
								collisions.push_back(result);

							} else {

								bodies1[collisionsCount] = &r1;
								bodies2[collisionsCount] = &r2;
								collisions[collisionsCount] = result;
							}
							collisionsCount ++;
						}
					}
				}

				forceRegistery.updateForces(fixedUpdateDt);

				for (int k=0; k<impulsInteration; k++){
					for (int i=0; i<collisionsCount; i++){
						int jSize = collisions[i].getContactPoints().size();

						for (int j=0; j<jSize; j++){
							rigidBody::RigidBody *r1 = bodies1[i];
							rigidBody::RigidBody *r2 = bodies2[i];

							applyImpulse(*r1, *r2, collisions[i]);
						}
					}
				}

				for (auto entity : mEntities){
					coordinator.GetComponent<rigidBody::RigidBody>(entity).physicsUpdate(fixedUpdateDt);
				}
			}

			void addEntity(ECS::Entity entity, bool gravity = true){
				if (gravity) forceRegistery.add(coordinator.GetComponent<rigidBody::RigidBody>(entity), this->gravity);

				coordinator.GetComponent<rigidBody::RigidBody>(entity).setTransform(&coordinator.GetComponent<components::Transform>(entity));
				coordinator.GetComponent<rigidBody::RigidBody>(entity).setCollider(coordinator.GetComponent<std::shared_ptr<physics2D::primitives::Collider2D>>(entity).get());
			}

		private:
			void applyImpulse(rigidBody::RigidBody &a, rigidBody::RigidBody &b, rigidBody::CollisionsManifold &m){
				float invertMass1 = a.getInvertMass();
				float invertMass2 = b.getInvertMass();
				float invertMasSum = invertMass1 + invertMass2;

				if (invertMasSum == 0.f) return;

				glm::vec2 relativeVel = b.getVelocity() - a.getVelocity();
				glm::vec2 relativeNormal = glm::normalize(m.getNormal());

				if (glm::dot(relativeVel, relativeNormal) > 0.0f) return;

				float e = std::min(a.cor(), b.cor());
				float numerator = -(1.f + e) * glm::dot(relativeVel, relativeNormal); 
				float j = numerator / invertMasSum;

				if (!m.getContactPoints().empty() && j != 0.0f){
					j /= static_cast<float>(m.getContactPoints().size());
				}

				glm::vec2 impulse = relativeNormal * j;
				a.setVelocity(a.getVelocity() + impulse * a.getInvertMass() * -1.f);
				b.setVelocity(b.getVelocity() + impulse * b.getInvertMass());
			}

			ECS::Coordinator &coordinator;
			forces::ForceRegistery forceRegistery;
			forces::Gravity2D gravity;

			float fixedUpdateDt;

			std::vector<rigidBody::RigidBody*> bodies1;
			std::vector<rigidBody::RigidBody*> bodies2;
			std::vector<rigidBody::CollisionsManifold> collisions;
			std::size_t collisionsCount = 0;
			std::size_t collisionsPoolSize = 1000;
			int impulsInteration = 6;
	};
}