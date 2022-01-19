#pragma once

#include "physics2D/forces/ForceRegistration.hpp"

// std
#include <list>
#include <memory>

namespace physics2D::forces{
	class ForceRegistery{
		public:
			ForceRegistery(){}

			void add(rigidBody::RigidBody &body, ForceGenerator &fg){
				registery.push_back(ForceRegistration(fg, body));
			}

			void remove(rigidBody::RigidBody &body, ForceGenerator &fg){
				registery.remove(ForceRegistration(fg, body));
			}

			void clear(){
				registery.clear();
			}

			void updateForces(float dt){
				for (ForceRegistration &fr : registery){
					fr.getForceGenerator().updateForce(fr.getBody(), dt);
				}
			}

			void zeroForces(){
				for (ForceRegistration &fr : registery){
					// fr.getBody().zeroForces();
				}
			}

		private:
			std::list<ForceRegistration> registery;
	};
}