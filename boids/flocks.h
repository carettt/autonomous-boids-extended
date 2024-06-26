#pragma once

#include "boid.h"
#include "channel.h"

#include <syncstream>
#include <atomic>
#include <barrier>
#include <CL/sycl.hpp>

// Struct to get kernel solutions
struct VisibleBoid {
    int visibleId = NULL;
    int lookingId = NULL;
};

class Flock {
public:
    // Array declaration
    static const int size = 500;
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
    void look(int index);
    void forget(int index);

    // TODO inter-thread communication to avoid recalculating collisions!
    // Update function
    virtual void update(double deltaTime);
};

class NaiveCPUFlock : public Flock {
private:
    std::vector<std::thread> flockThreads;
    std::barrier<> ready;

public:
    template<typename F>
    NaiveCPUFlock(F dna, float sWeight, float cWeight, float aWeight, std::mt19937 gen, std::shared_ptr<sf::RenderWindow> window, unsigned int threads) :
        Flock(dna, sWeight, cWeight, aWeight, gen, window), ready(threads), flockThreads(threads) {}

    // Update functions
    void boundedUpdate(int lower, int upper);
    void update(double deltaTime);
};

// Intermediate class between CPUFlock and Flock (originally planned to use chunks in GPU implementation)
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

//! PLEASE DONT MARK THIS, PLEASE MARK NaiveCPUFlock INSTEAD
// Original CPU implementation dividing boids into chunks (ChunkedFlock), reducing amount of boids needed to be checked by looking boids
// I wasn't able to figure this out and complete it in time, any feedback would be appreciated...
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
                this->lookThreads.emplace_back([&]() {
                    for (;;) {
                        this->lookSync.arrive_and_wait();

                        //for (Boid* boid : this->chunks[i][j].owned) {
                        //    boid->visible.clear();
                        //}

                        //int chunkWidth = this->chunks[0][0].bottomRight.x - this->chunks[0][0].topLeft.x;
                        //int chunkHeight = this->chunks[0][0].bottomRight.y - this->chunks[0][0].topLeft.y;

                        //for (Boid* boid : this->chunks[i][j].owned) {
                        //    // Get visibility radius and chunk dimensions
                        //    int visibilityRadius = std::ceil(boid->visibility * boid->radius);

                        //    // Define the x and y chunk visibility radius and add 1 for for smoothing
                        //    int chunkXRadius = (visibilityRadius / chunkWidth) + (visibilityRadius % chunkWidth != 0) + 1;
                        //    int chunkYRadius = (visibilityRadius / chunkHeight) + (visibilityRadius % chunkHeight != 0) + 1;

                        //    std::list<std::pair<int, int>> offsets = this->generateOffsets(chunkXRadius, chunkYRadius);
                        //    std::list<std::unique_ptr<Chunk>> adjacent = this->getAdjacentChunks(std::make_pair(i, j), offsets);

                        //    for (std::unique_ptr<Chunk>& chunk : adjacent) {
                        //        for (Boid* other : chunk->owned) {
                        //            float relativeDistance = sfvec::getToroidalDistance(boid->position, other->position, this->window->getSize());

                        //            if (relativeDistance < visibilityRadius && other->id != boid->id) {
                        //                boid->visible.push_back(*other);
                        //            }
                        //        }
                        //    }
                        //}

                        this->threadSync.arrive_and_wait();
                    }
                });

                this->updateThreads.emplace_back([&]() {
                    for (;;) {
                        this->threadSync.arrive_and_wait();

                        for (Boid* boid : this->chunks[i][j].owned) {
                            boid->update(this->window->getSize(), this->w, this->gen);
                        }
                        this->updateSync.arrive_and_wait();
                    }
                });
            }
        }
    }

    //void look(int i, const sf::Vector2u& dimensions);
    void update(double deltaTime);
};

class GPUFlock : public Flock { 
private:
    sycl::queue q;

public:
    template<typename F>
    GPUFlock(F dna, float sWeight, float cWeight, float aWeight, std::mt19937 gen, std::shared_ptr<sf::RenderWindow> window) :
        Flock(dna, sWeight, cWeight, aWeight, gen, window) {}

    void flattenBoids(FlatBoid* output) {
        for (int i = 0; i < this->size; i++) {
            output[i] = this->boids[i].flatten();
        }
    }

    void setDevice(sycl::device d) {
        this->q = sycl::queue(d);
    }

    void update(double deltaTime);
};