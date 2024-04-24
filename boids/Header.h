#pragma once

#include <SFML/Graphics.hpp>

#include "sfvec.h"

#include <list>
#include <chrono>
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

class Chunk;

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
    friend class Flock;
    friend class ChunkedFlock;
    friend class CPUFlock;
};

class Chunk {
private:
    // Store chunk border as top left and bottom right points
    sf::Vector2f topLeft;
    sf::Vector2f bottomRight;

    // Store chunk index/position
    sf::Vector2u position;

    // List of boids in chunk
    std::list<std::shared_ptr<Boid>> owned;
public:
    Chunk() : topLeft(0, 0), bottomRight(0, 0) {};

    Chunk(sf::Vector2f topLeft, sf::Vector2f bottomRight) : topLeft(topLeft), bottomRight(bottomRight) {};

    friend class ChunkedFlock;
    friend class CPUFlock;
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
    Flock(F dna, float sWeight, float cWeight, float aWeight);

    // Visibility update functions
    virtual void look(int index, const sf::Vector2u& dimensions);
    void forget(int index);

    // TODO inter-thread communication to avoid recalculating collisions!
    // Update function
    virtual void update(std::shared_ptr<sf::RenderWindow> window, std::mt19937& gen, float deltaTime);
};

class ChunkedFlock : public Flock {
protected:
    std::vector<std::vector<Chunk>> chunks;

    // Adjacency offsets
    std::pair<int, int> offsets[8] = {
        { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },  // Adjacent chunks in horizontal and vertical directions
        { -1, -1 }, { 1, 1 } // Diagonally adjacent chunks
    };
public:
    //Constructor
    template<typename F>
    ChunkedFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions);

    void localizeBoids();
    int countBoids();

    std::list<std::pair<int, int>> generateOffsets(int xRadius, int yRadius);

    std::list<std::unique_ptr<Chunk>> getAdjacentChunks(std::pair<int, int> index, std::list<std::pair<int, int>> offsets);
};

class CPUFlock : public ChunkedFlock {
private:
    std::list<std::thread> threads;

public:
    template<typename F>
    CPUFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions);

    void look(int i, const sf::Vector2u& dimensions);
};