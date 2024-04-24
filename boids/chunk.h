#pragma once

#include <SFML/System.hpp>

#include "flock.h"

class Boid;

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

    friend class flocks::ChunkedFlock;
    friend class flocks::CPUFlock;
};