#include "physics2D/rigidBody/IntersectionDetector2D.hpp"

// libs
#include <SDL2/SDL.h>

// std
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>

static constexpr float minExecutionTime = 16;

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


	// the main loop
	auto start = std::chrono::high_resolution_clock::now();
	float dt = 0;

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
				
				default:
					break;
			}
		}

		// update
		

		// rendering
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		SDL_RenderClear(renderer);


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