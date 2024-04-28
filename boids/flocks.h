#pragma once

#include "boid.h"
#include "channel.h"

class Flock {
public:
    // Array declaration
    static const int size = 100;
    Boid boids[size];

    // Steering force weights
    Weights w;

    // Constructor
    template<typename F>
    inline Flock(F dna, float sWeight, float cWeight, float aWeight) : w(sWeight, cWeight, aWeight) {
        // Loop through empty array
        for (int i = 0; i < this->size; i++) {
            // Assign i-th element to result of 'DNA' callback function, passing the index as an argument
            this->boids[i] = dna(i);
            this->boids[i].defaultTopSpeed = this->boids[i].topSpeed;
        }
    }

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
public:
    //Constructor
    template<typename F>
    inline ChunkedFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions) :
        Flock(dna, sWeight, cWeight, aWeight)
    {
        // Initialize chunks vector
        chunks = std::vector<std::vector<Chunk>>(splits, std::vector<Chunk>(splits, Chunk()));

        // Divide window area into splits^2 chunks
        for (int i = 0; i < splits; i++) {
            for (int j = 0; j < splits; j++) {
                sf::Vector2f topLeft((dimensions.x / splits) * i, (dimensions.y / splits) * j);
                sf::Vector2f bottomRight((dimensions.x / splits) * (i + 1), (dimensions.y / splits) * (j + 1));
                chunks[i][j] = Chunk(topLeft, bottomRight, std::make_pair(i, j));
            }
        }
    }

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
    inline CPUFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions) :
        ChunkedFlock(dna, sWeight, cWeight, aWeight, splits, dimensions) {
        for (int i = 0; i < pow(splits, 2); i++) {
            threads.push_back(std::thread());
        }
    }

    void look(int i, const sf::Vector2u& dimensions);
};