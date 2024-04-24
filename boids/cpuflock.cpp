#include "flock.h"
#include "chunk.h"

// CPU Parallelised

template<typename F>
flocks::CPUFlock::CPUFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions) :
    ChunkedFlock(dna, sWeight, cWeight, aWeight, splits, dimensions) {
    for (int i = 0; i < pow(splits, 2); i++) {

    }
}

void flocks::CPUFlock::look(int i, const sf::Vector2u& dimensions) {
    // Update i-th boid's visible list

    // Get visibility chunk radius
    int visibilityRadius = std::ceil(this->boids[i].visibility * this->boids[i].radius);
    int chunkWidth = this->chunks[0][0].bottomRight.x - this->chunks[0][0].topLeft.x;
    int chunkHeight = this->chunks[0][0].bottomRight.y - this->chunks[0][0].topLeft.y;

    int chunkXRadius = (visibilityRadius / chunkWidth) + (visibilityRadius % chunkWidth != 0);
    int chunkYRadius = (visibilityRadius / chunkHeight) + (visibilityRadius % chunkHeight != 0);


}