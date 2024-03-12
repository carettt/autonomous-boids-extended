#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <math.h>
#include <list>
#include <iostream>

#include <random>
#include <vector>
#include <chrono>
#include <thread>

// Global constant to convert from radians to degrees
const float TO_DEGREES = 180.f / M_PI;

// Vector math functions for SFML vectors
namespace sfvec {
	// Static constants for zero vectors
	static const sf::Vector2f ZEROF = sf::Vector2f(0.f, 0.f);
	static const sf::Vector2i ZEROI = sf::Vector2i(0, 0);

	// Print vector for debug
	template<typename T>
	void println(const sf::Vector2<T>& v) {
		std::cout << "{" << v.x << ", " << v.y << "}" << std::endl;
	}

	template<typename T>
	int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}

	// Return dot product of two vectors
	template<typename T>
	float dot(sf::Vector2<T> a, sf::Vector2<T> b) {
		return (a.x * b.x) + (a.y * b.y);
	}

	// Get distance between two vectors
	template<typename T>
	float getDistance(const sf::Vector2<T>& a, const sf::Vector2<T>& b) {
		return sqrt(pow((b.x - a.x), 2) + pow(b.y - a.y, 2));
	}

	template<typename T>
	sf::Vector2<T> getRelativeToroidalPosition(const sf::Vector2<T>& of, const sf::Vector2<T>& to, const sf::Vector2u& dimensions) {
		int m = -sgn(of.x - to.x);
		int n = -sgn(of.y - to.y);

		sf::Vector2<T> avatars[4];
		avatars[0] = of;
		avatars[1] = of + sf::Vector2<T>(m * dimensions.x, 0);
		avatars[2] = of + sf::Vector2<T>(0, n * dimensions.y);
		avatars[3] = of + sf::Vector2<T>(m * dimensions.x, n * dimensions.y);

		int closest = 0;

		for (int i = 1; i < 4; i++) {
			float distance = sfvec::getDistance(to, avatars[i]);
			float closestDistance = sfvec::getDistance(to, avatars[closest]);

			if (distance < closestDistance) {
				closest = i;
			}
		}

		return avatars[closest];
	}

	// Get distance between two vectors in a toroidal space
	template<typename T>
	float getToroidalDistance(const sf::Vector2<T>& a, const sf::Vector2<T>& b, const sf::Vector2u& dimensions) {
		// Get distances of vector components in each dimension
		float dx = abs(b.x - a.x);
		float dy = abs(b.y - a.y);

		// If the distance is greater than half the dimension's total length,
		// it is shorter to go the opposite direction, thus the real distance is the dimension - previous distance
		if (dx > (dimensions.x / 2)) {
			dx = dimensions.x - dx;
		}

		if (dy > (dimensions.y / 2)) {
			dy = dimensions.y - dy;
		}

		// Return true distance
		return sqrt(pow(dx, 2) + pow(dy, 2));
	}

	// Get magnitude of vector
	template<typename T>
	float getMagnitude(const sf::Vector2<T>& v) {
		return sqrt(pow(v.x, 2) + pow(v.y, 2));
	}

	// Normalize vector
	template<typename T>
	sf::Vector2<T> normalize(const sf::Vector2<T>& v) {
		sf::Vector2<T> normalized = v / sfvec::getMagnitude(v);

		return normalized;
	}

	// Clamp vector magnitude to max
	template<typename T>
	sf::Vector2<T> clampMagnitude(const sf::Vector2<T>& v, T max) {
		return sfvec::getMagnitude(v) > max ? sfvec::normalize(v) * max : v;
	}

	// Get rotation of vector
	template<typename T>
	float getRotation(const sf::Vector2<T>& v) {
		// Handle v.x being 0, to avoid zero division
		if (v.x == 0) {
			if (v.y > 0) {
				return 90.f;
			}
			else if (v.y < 0) {
				return 270.f;
			}
			else {
				return NAN;
			}
		}
		else {
			return atan((-1 * v.y) / v.x) * TO_DEGREES;
		}
	}
}

template<typename T>
T clamp(T val, T max) {
	return val > max ? max : val;
}

// Structure to hold steering force weights
struct Weights {
	float sWeight;
	float cWeight;
	float aWeight;
};

class Boid {
private:
	// List holding visible boids
	std::list<Boid> visible;

	// Boid information
	sf::Vector2f position;
	sf::Vector2f velocity;
	float defaultTopSpeed;
	float topSpeed;
	float visibility;

	// Steering forces
	sf::Vector2f separation;
	sf::Vector2f cohesion;
	sf::Vector2f alignment;

	// Leadership
	bool leader = false;
	float leaderDuration = 1500;
	float eccentricity;

	std::chrono::steady_clock::time_point leaderTimerStart;
	std::chrono::steady_clock::time_point leaderTimerStop;

	// Render information
	float radius;
	sf::CircleShape triangle;
public:
	// Default constructor
	Boid() {
		this->visibility = 5.f;
		this->topSpeed = 25.f;
		this->position = sf::Vector2f(0, 0);
		this->velocity = sf::Vector2f(0, 0);
		this->radius = 5;
		this->triangle = sf::CircleShape(radius, 3);
		this->triangle.setPosition(this->position);
		this->triangle.setFillColor(sf::Color::White);
	}

	// Constructor
	Boid(float x, float y, float radius, float topSpeed, sf::Vector2f v = sf::Vector2f(0, 0), float visibility = 5.f) {
		this->visibility = visibility;
		this->topSpeed = topSpeed;
		this->position = sf::Vector2f(x, y);
		this->velocity = v;
		this->radius = radius;
		this->triangle = sf::CircleShape(radius, 3);
		this->triangle.setOrigin(radius, radius);
		this->triangle.setPosition(this->position);
		this->triangle.setFillColor(sf::Color::White);
	}

	// Steering forces
	void calculateSeparation(const sf::Vector2u& dimensions);
	void calculateCohesion(const sf::Vector2u& dimensions);
	void calculateAlignment();

	// Leadership
	float escapeAcceleration(float t);

	void calculateEccentricity(const sf::Vector2u& dimensions);
	void attemptEscape(std::mt19937& gen, sf::Vector2u dimensions);

	// Update functions
	void update(const sf::Vector2u& dimensions, Weights w, std::mt19937& gen);
	void draw(std::shared_ptr<sf::RenderWindow> window, float deltaTime);

	// Allow Flocks to access private/protected members
	friend class Flock;
};

class Flock {
public:
	// Array declaration
	static const int size = 100;
	Boid boids[size];

	// Steering force weights
	Weights w;

	// Constructor
	template<typename F>
	Flock(F dna, float sWeight, float cWeight, float aWeight) {
		// Assign weights to structure
		this->w.sWeight = sWeight;
		this->w.cWeight = cWeight;
		this->w.aWeight = aWeight;

		// Loop through empty array
		for (int i = 0; i < this->size; i++) {
			// Assign i-th element to result of 'DNA' callback function, passing the index as an argument
			this->boids[i] = dna(i);
			this->boids[i].defaultTopSpeed = this->boids[i].topSpeed;
		}
	}

	// Visibility update functions
	void look(int index, const sf::Vector2u& dimensions);
	void forget(int index);

	// TODO inter-thread communication to avoid recalculating collisions!
	// Update function
	void update(std::shared_ptr<sf::RenderWindow> window, std::mt19937& gen, float deltaTime);
};