#include <print>
#include <vector>
#include <string>
#include <random>

#include "Physics.hpp"

constexpr	int		window_width	= 1000;
constexpr	int		window_height	= 800;
constexpr	int		target_fps		= 60;
const		char*	window_title	= "Physics Sandbox 2D";

std::vector<std::string> args2vec(int argc, char* argv[]) {
	std::vector<std::string> args(argc - 1);
	for (int i = 1; i < argc; i++) {
		args[i - 1] = std::string(argv[i]);
	}
	return args;
}

int main(int argc, char* argv[]) {
	const auto args = args2vec(argc, argv);

	InitWindow(window_width, window_height, window_title);
	SetTargetFPS(target_fps);

	std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<double> xDist(50, window_width - 50);
	std::uniform_real_distribution<double> yDist(0, window_height - 50);
	std::uniform_real_distribution<double> vDist(-200.0, 200.0);

	constexpr int BALL_COUNT = 1000;

	PhysicsWorld world;

	for (int i = 0; i < BALL_COUNT; i++) {
		RigidBody b;
		b.dynamic = true;
		b.gravity = true;
		b.mass = 1.0;
		b.radius = 5.0;

		b.position = {
			xDist(rng),
			yDist(rng)
		};

		b.velocity = {
			vDist(rng),
			vDist(rng)
		};

		world.addBody(b);
	}

	constexpr double FLOOR_RADIUS = 5.0;
	constexpr double FLOOR_Y = window_height - FLOOR_RADIUS; // just above y = 0

	for (double x = FLOOR_RADIUS; x < window_width; x += FLOOR_RADIUS * 2.0) {
		RigidBody floor;
		floor.dynamic = false;   // static
		floor.gravity = false;
		floor.mass = 0.0;        // ignored
		floor.radius = FLOOR_RADIUS;

		floor.position = { x, FLOOR_Y };
		floor.velocity = { 0.0, 0.0 };

		world.addBody(floor);
	}


	constexpr int rad = 5;

	while (!WindowShouldClose()) {
		BeginDrawing();
			
			ClearBackground(BLACK);
			world.tick(GetFrameTime());

			for (const auto& body : world.bodies) {
				DrawCircle(body.position.x, body.position.y, rad, WHITE);
				//std::println("Body Position X: {} Y: {}", body.position.x, body.position.y);
			}

		EndDrawing();

		
	}

	CloseWindow();
}