#include "physics2D/PhysicsSystem2D.hpp"
#include "physics2D/primitives/Collider2D.hpp"
#include "ECS.hpp"

// libs
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>

// std
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

static constexpr float minExecutionTime = 16;

void function(){

}

int main(int argc, char **argv){
	std::cout << "version : " << VERSION << std::endl;

	SDL_Window *window;
	SDL_Renderer *renderer;

	// init SDL2
	if (SDL_Init(SDL_INIT_VIDEO) != 0){
		std::cerr << "ERROR :: SDL_Init : " << SDL_GetError() << std::endl;
		return EXIT_FAILURE;
	}

	window = SDL_CreateWindow("2d physics", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1080, 720, SDL_WINDOW_SHOWN);

	if (!window){
		std::cerr << "ERROR :: SDL_CreateWindow : " << SDL_GetError() << std::endl;
		SDL_Quit();
		return EXIT_FAILURE;
	}

	renderer = SDL_CreateRenderer(window, -1, 0);

	if (!renderer){
		std::cerr << "ERROR :: SDL_CreateRenderer : " << SDL_GetError() << std::endl;
		SDL_DestroyWindow(window);
		SDL_Quit();
		return EXIT_FAILURE;
	}

	// ECS
	ECS::Coordinator coordinator;
	coordinator.RegisterComponent<components::Transform>();
	coordinator.RegisterComponent<physics2D::rigidBody::RigidBody>();
	coordinator.RegisterComponent<physics2D::primitives::Collider2D>();

	auto physicsSystem = coordinator.RegisterSystem<physics2D::PhysicsSystem>(0.16, coordinator);
	ECS::Signature signature;
	signature.set(coordinator.GetComponentType<components::Transform>());
	signature.set(coordinator.GetComponentType<physics2D::rigidBody::RigidBody>());
	coordinator.SetSystemSignature<physics2D::PhysicsSystem>(signature);
	
	std::vector<components::Transform*> transforms;

	// the main loop
	auto start = std::chrono::high_resolution_clock::now();
	float dt = 0;
	glm::vec2 mousePos;

	bool launched = true;
	while (launched){
		start = std::chrono::high_resolution_clock::now();

		// events
		SDL_Event e;
		while (SDL_PollEvent(&e)){
			switch (e.type){
				case SDL_QUIT:
					launched = false;
					break;
				
				case SDL_MOUSEBUTTONDOWN:
					switch (e.button.button){
						case SDL_BUTTON_LEFT:
							ECS::Entity e = coordinator.CreateEntity();

							physics2D::rigidBody::RigidBody b;
							b.setMass(2.f);
							b.setTransform(mousePos);

							coordinator.AddComponent<components::Transform>(e);
							coordinator.AddComponent<physics2D::rigidBody::RigidBody>(e, b);
							transforms.push_back(&coordinator.GetComponent<components::Transform>(e));
							physicsSystem->addEntity(e);
							break;
					}
					break;
				
				case SDL_MOUSEMOTION:
					mousePos = glm::vec2(e.motion.x, e.motion.y);
				
				default:
					break;
			}
		}

		// rendering
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);

		physicsSystem->update(dt);

		for (auto &t : transforms){
			circleRGBA(renderer, t->getPosition().x, t->getPosition().y, 30, 255, 0, 0, 255);
		}

		SDL_RenderPresent(renderer);

		// fps limiter
		float execTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - start).count();
		float waitTime = execTime > minExecutionTime ? minExecutionTime : execTime;
		SDL_Delay(minExecutionTime - waitTime);
		dt = std::chrono::duration<float, std::chrono::seconds::period>(std::chrono::high_resolution_clock::now() - start).count();
	}

	// clear
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return EXIT_SUCCESS;
}