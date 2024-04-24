#pragma once

#include "boid.h"

namespace flocks {
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
}