//Polianski, Michael
// System Headers
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
// Project Headers
#include "nbody.h"
#include <vector>
#include <thread>



//#define GRAPHICS
#ifdef GRAPHICS
	#include <SFML/Window.hpp>
	#include <SFML/Graphics.hpp>
#endif

// Number of particles
 //#define SMALL
#define LARGE
//#define MASSIVE

#if defined(SMALL)
	const int N = 1000;
#elif defined(LARGE)
	const int N = 5000;
#elif defined(MASSIVE)
	const int N = 10000;
#endif


// Constants
const double min2 = 2.0;
const double G = 1 * 10e-10;
const double dt = 0.01;
const int NO_STEPS = 500;

// Size of Window/Output image
const int width = 1920;
const int height = 1080;

// Bodies
body bodies[N];

//Main acceleration computation turned into a fucntion which threads process
void compute(int start, int end, std::vector<vec2>& acc) {
	
	for(int i = start; i < end; ++i) {
		
		// For each following body
		for(int j = i+1; j < N; ++j) {
			// Difference in position
			
			vec2 dx = bodies[i].pos - bodies[j].pos;

			// Normalised difference in position
			vec2 u = normalise(dx);

			// Calculate distance squared
			double d2 = length2(dx);

			// If greater than minimum distance
			if(d2 > min2) {
				// Smoothing factor for particles close to minimum distance
				double x = smoothstep(min2, 2 * min2, d2);

				// Force between bodies
				double f = -G*bodies[i].mass*bodies[j].mass / d2;

				// Add to acceleration
				acc[i] += (u * f / bodies[i].mass) * x;
				acc[j] -= (u * f / bodies[j].mass) * x;
			}
		}
	}
}

// Update Nbody Simulation
void update() {
	// Acceleration
	//
	//Create a named acc with a size of N and instialises each element to vec2
	std::vector<vec2> acc(N, vec2(0, 0)); 
	//Get the total amount of threads that are availaible
	int total_threads = std::thread::hardware_concurrency();
	//Create a vector named threads
	std::vector<std::thread> threads;
	//Evenly split the amount of bodies accross each thread
	int distribution = N / total_threads;
	//A two diemensional vector where the out vector has a size of total_threads and the inner of size N
	std::vector<std::vector<vec2>> local_acc(total_threads, std::vector<vec2>(N, vec2(0,0)));
	// For each body

	
	int end;
	for (int indx = 0; indx < total_threads; indx++) {
		//aquires the starting index for the first body the thread will process
		int start = indx * distribution;
		//checks if the current thread is the last one, if it is then the end becomes N if else then the end gets the distributation starting from start
		if (indx == total_threads - 1) {
			end = N;
		} else {
			end = start + distribution;
		}
		//creates a new thread and adds to the threds vector it captures variables by reference and sends them to the compute function
		//Uses the emplace_back insteads of push_back since it constructs objects directly into memory bypassing the need to temporary objects
		threads.emplace_back([&, indx, start, end](){compute(start, end, local_acc[indx]);});
	}

	//This makes sure each thread waits for the worker threads to finish before continuing
	for (int indx = 0; indx < threads.size(); indx++) {
		threads[indx].join();
	}
	//This combines all the accelerations calculated from each of the threads
	for (int indx = 0; indx < total_threads; indx++) {
		for (int i = 0; i < N; i++) {
			acc[i] = acc[i] + local_acc[indx][i];
		}
	}


	// For each body
	for(int i = 0; i < N; ++i) {
		// Update Position
		bodies[i].pos += bodies[i].vel * dt;

		// Update Velocity
		bodies[i].vel += acc[i] * dt;
	}
}



// Initialise NBody Simulation
void initialise() {
	// Create a central heavy body (sun)
	bodies[0] = body(width/2, height/2, 0, 0, 1e15, 5);

	// For each other body
	for(int i = 1; i < N; ++i) {
		// Pick a random radius, angle and calculate velocity
		double r = (uniform() + 0.1) * height/2;
		double theta = uniform() * 2 * M_PI;
		double v = sqrt(G * (bodies[0].mass + bodies[i].mass) / r);

		// Create orbiting body
		bodies[i] = body(width/2 + r * cos(theta), height/2 + r * sin(theta), -sin(theta) * v, cos(theta)*v, 1e9, 2);
	}
}

#ifdef GRAPHICS
	// Main Function - Graphical Display
	int main() {
		// Create Window
		sf::ContextSettings settings;
		settings.antialiasingLevel = 1;
		sf::RenderWindow window(sf::VideoMode(width, height), "NBody Simulator", sf::Style::Default, settings);

		// Initialise NBody Simulation
		initialise();
		int i = 0;
		// run the program as long as the window is open
		while (window.isOpen()) {
			// check all the window's events that were triggered since the last iteration of the loop
			sf::Event event;
			while (window.pollEvent(event)) {
				// "close requested" event: we close the window
				if (event.type == sf::Event::Closed) {
					window.close();
				}
			}

			if(i < NO_STEPS) {
				// Update NBody Simluation
				update();
				i++;
			}

			// Clear the window with black color
			window.clear(sf::Color::Black);

			// Render Objects
			for(int i = 0; i < N; ++i) {
				// Create Circle
				sf::CircleShape shape(bodies[i].radius);
				shape.setFillColor(sf::Color(255, 0, 0));
				shape.setPosition(bodies[i].pos.x, bodies[i].pos.y);
				
				// Draw Object
				window.draw(shape);
			}

			// Display Window
			window.display();
		}
	}
#else
	// Main Function - Benchmark
	int main() {
		std:: cout << std::endl;
		// Initialise NBody Simulation
		initialise();

		// creates a thread pool with a number of threads equal to the available threads 

		// Get start time
		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		// Run Simulation
		
		for(int i = 0; i < NO_STEPS; i++) {
			// Update NBody Simluation
			update();
		}

		// Get end time
		std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

		// Generate output image
		unsigned char *image = new unsigned char[width * height * 3];
		memset(image, 0, width * height * 3);

		// For each body
		for(int i = 0; i < N; ++i) {
			// Get Position
			vec2 p = bodies[i].pos;

			// Check particle is within bounds
			if(p.x >= 0 && p.x < width && p.y >= 0 && p.y < height) {
				// Add a red dot at body
				image[((((int)p.y * width) + (int)p.x) * 3)] = 255;
			}
		}

		// Write position data to file
		char data_file[200];
		sprintf(data_file, "output%i.dat", N);
		write_data(data_file, bodies, N);

		// Write image to file
		char image_file[200];
		sprintf(image_file, "output%i.png", N);
		write_image(image_file, bodies, N, width, height);

		// Check Results
		char reference_file[200];
		sprintf(reference_file, "reference%i.dat", N);
		calculate_maximum_difference(reference_file, bodies, N);

		// Time Taken
		std::cout << "Time Taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000000.0 << std::endl;
	}
#endif