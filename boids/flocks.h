#pragma once

#include "boid.h"
#include "channel.h"

#include <syncstream>
#include <atomic>
#include <barrier>

class Flock {
public:
    // Array declaration
    static const int size = 100;
    Boid boids[size];

    // Steering force weights
    Weights w;

    std::mt19937 gen;

    std::shared_ptr<sf::RenderWindow> window;

    // Constructor
    template<typename F>
    Flock(F dna, float sWeight, float cWeight, float aWeight, std::mt19937 gen, std::shared_ptr<sf::RenderWindow> window) :
        w(sWeight, cWeight, aWeight), gen(gen), window(window) {
        // Loop through empty array
        for (int i = 0; i < this->size; i++) {
            // Assign i-th element to result of 'DNA' callback function, passing the index as an argument
            this->boids[i] = dna(i);
            this->boids[i].id = i;
            this->boids[i].defaultTopSpeed = this->boids[i].topSpeed;
        }
    }

    // Visibility update functions
    virtual void look(int index);
    void forget(int index);

    // TODO inter-thread communication to avoid recalculating collisions!
    // Update function
    virtual void update(float deltaTime);
};

class ChunkedFlock : public Flock {
protected:
    std::vector<std::vector<Chunk>> chunks;
public:
    //Constructor
    template<typename F>
    inline ChunkedFlock(F dna, float sWeight, float cWeight, float aWeight, std::mt19937 gen, std::shared_ptr<sf::RenderWindow> window, const int& splits) :
        Flock(dna, sWeight, cWeight, aWeight, gen, window)
    {
        // Initialize chunks vector
        chunks = std::vector<std::vector<Chunk>>(splits, std::vector<Chunk>(splits, Chunk()));

        // Divide window area into splits^2 chunks
        for (int i = 0; i < splits; i++) {
            for (int j = 0; j < splits; j++) {
                sf::Vector2f topLeft((this->window->getSize().x / splits) * i, (this->window->getSize().y / splits) * j);
                sf::Vector2f bottomRight((this->window->getSize().x / splits) * (i + 1), (this->window->getSize().y / splits) * (j + 1));
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
    std::list<std::thread> lookThreads;
    std::list<std::thread> updateThreads;

    std::barrier<> threadSync;
    std::barrier<> updateSync;
    std::barrier<> lookSync;
public:
    template<typename F>
    CPUFlock(F dna, float sWeight, float cWeight, float aWeight, std::mt19937 gen, std::shared_ptr<sf::RenderWindow> window, const int& splits) :
        ChunkedFlock(dna, sWeight, cWeight, aWeight, gen, window, splits), threadSync(pow(this->chunks.size(), 2) * 2), updateSync(pow(this->chunks.size(), 2) + 1), lookSync(pow(this->chunks.size(), 2) + 1)
    {
        for (int i = 0; i < splits; i++) {
            for (int j = 0; j < splits; j++) {
                this->lookThreads.emplace_back([this, i, j]() {
                    for (;;) {
                        this->lookSync.arrive_and_wait();

                        for (std::reference_wrapper<Boid> boid : this->chunks[i][j].owned) {
                            boid.get().visible.clear();
                        }

                        int chunkWidth = this->chunks[0][0].bottomRight.x - this->chunks[0][0].topLeft.x;
                        int chunkHeight = this->chunks[0][0].bottomRight.y - this->chunks[0][0].topLeft.y;

                        for (std::reference_wrapper<Boid> boid : this->chunks[i][j].owned) {
                            // Get visibility radius and chunk dimensions
                            int visibilityRadius = std::ceil(boid.get().visibility * boid.get().radius);

                            // Define the x and y chunk visibility radius and add 1 for for smoothing
                            int chunkXRadius = (visibilityRadius / chunkWidth) + (visibilityRadius % chunkWidth != 0) + 1;
                            int chunkYRadius = (visibilityRadius / chunkHeight) + (visibilityRadius % chunkHeight != 0) + 1;

                            std::list<std::pair<int, int>> offsets = this->generateOffsets(chunkXRadius, chunkYRadius);
                            std::list<std::unique_ptr<Chunk>> adjacent = this->getAdjacentChunks(std::make_pair(i, j), offsets);

                            for (std::unique_ptr<Chunk>& chunk : adjacent) {
                                for (std::reference_wrapper<Boid> other : chunk->owned) {
                                    float relativeDistance = sfvec::getToroidalDistance(boid.get().position, other.get().position, this->window->getSize());

                                    if (relativeDistance < visibilityRadius && other.get().id != boid.get().id) {
                                        boid.get().visible.push_back(other.get());
                                    }
                                }
                            }
                        }

                        this->threadSync.arrive_and_wait();
                    }
                });

                this->updateThreads.emplace_back([this, i, j]() {
                    for (;;) {
                        this->threadSync.arrive_and_wait();

                        for (std::reference_wrapper<Boid> boid : this->chunks[i][j].owned) {
                            boid.get().update(this->window->getSize(), this->w, this->gen);
                        }
                        this->updateSync.arrive_and_wait();
                    }
                });
            }
        }
    }

    //void look(int i, const sf::Vector2u& dimensions);
    void update(float deltaTime);
};