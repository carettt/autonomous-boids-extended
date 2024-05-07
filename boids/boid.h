#pragma once

#include "sfvec.h"
#include <optional>

// TODO use member initialization list on Boid and Flock constructors

// Structure to hold steering force weights
struct Weights {
    float sWeight;
    float cWeight;
    float aWeight;

    Weights(float sWeight, float cWeight, float aWeight) :
        sWeight(sWeight), cWeight(cWeight), aWeight(aWeight) {}
};

struct FlatBoid {
    float x;
    float y;
    float visibilityRadius;

    int id;
};

class Chunk;

class Boid {
protected:
    int id;

    // List holding visible boids
    std::list<Boid*> visible;

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
    void draw(std::shared_ptr<sf::RenderWindow> window, double deltaTime);

    // Flatten boid into position array
    FlatBoid flatten() {
        return { this->position.x, this->position.y, this->visibility * this->radius, this->id };
    }

    // Allow Flocks to access private/protected members
    friend class Flock;
    friend class NaiveCPUFlock;
    friend class ChunkedFlock;
    friend class CPUFlock;
    friend class GPUFlock;
};

class Chunk {
private:
    // Store chunk border as top left and bottom right points
    sf::Vector2f topLeft;
    sf::Vector2f bottomRight;

    // Store chunk index/position
    std::pair<int, int> index;

    // List of boids in chunk
    std::list<Boid*> owned;
public:
    Chunk() : topLeft(0, 0), bottomRight(0, 0), index(std::make_pair(0, 0)) {};

    Chunk(sf::Vector2f topLeft, sf::Vector2f bottomRight, std::pair<int, int> index) :
        topLeft(topLeft), bottomRight(bottomRight), index(index) {};

    friend class ChunkedFlock;
    friend class CPUFlock;
};