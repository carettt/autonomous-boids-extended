#pragma once

#include <SFML/Graphics.hpp>
#include <optional>

#include "sfvec.h"

#include "flock.h"

// Forward declarations
class Chunk;

// TODO use member initialization list on Boid and Flock constructors

// Structure to hold steering force weights
struct Weights {
    float sWeight;
    float cWeight;
    float aWeight;

    Weights(float sWeight, float cWeight, float aWeight) :
        sWeight(sWeight), cWeight(cWeight), aWeight(aWeight) {}
};

class Boid {
protected:
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

    // Chunk information
    std::optional<std::shared_ptr<Chunk>> currentChunk;

    // Render information
    float radius;
    sf::CircleShape triangle;
public:
    // Default constructor
    Boid();

    // Constructor
    Boid(float x, float y, float radius, float topSpeed, sf::Vector2f v = sfvec::ZEROF, float visibility = 5.f);

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
    friend class flocks::ChunkedFlock;
    friend class flocks::CPUFlock;
    friend class flocks::Flock;
};
