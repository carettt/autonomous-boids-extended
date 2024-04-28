#include "flocks.h"

void Flock::look(int i, const sf::Vector2u& dimensions) {
    // Update i-th boid's visible list

    // Loop through all boids
    for (int j = 0; j < this->size; j++) {
        float relativeDistance = sfvec::getToroidalDistance(this->boids[i].position, this->boids[j].position, dimensions);

        // If distance between i-th boid and j-th boid is less than the visibility factor * radius of self, j-th boid is visible
        if (relativeDistance < (this->boids[i].radius * this->boids[i].visibility) && i != j) {
            this->boids[i].visible.push_back(this->boids[j]);
        }
    }
}

void Flock::forget(int index) {
    // Clear all boids' visible lists
    this->boids[index].visible.clear();
}

void Flock::update(std::shared_ptr<sf::RenderWindow> window, std::mt19937& gen, float deltaTime) {
    // Call all necessary frametime functions on all boids

    // Loop through all boids
    for (int i = 0; i < this->size; i++) {
        // Update i-th boid's visible lists
        this->look(i, window->getSize());
        // Update i-th boid's forces
        this->boids[i].update(window->getSize(), this->w, gen);
        // Add boid to render queue
        this->boids[i].draw(window, deltaTime);
        // Clear all boids' visible lists
        this->forget(i);
    }
}

void ChunkedFlock::localizeBoids() {
    // Split boids into their respective chunks

    //Loop through all boids
    for (Boid& boid : this->boids) {
        // Loop through all chunks
        for (std::vector<Chunk>& row : this->chunks) {
            for (Chunk& chunk : row) {
                // If boid is within chunk bounds then the chunk 'owns' the boid
                if (boid.position.x >= chunk.topLeft.x &&
                    boid.position.x <= chunk.bottomRight.x &&
                    boid.position.y >= chunk.topLeft.y &&
                    boid.position.y <= chunk.bottomRight.y)
                {
                    chunk.owned.push_back(std::make_shared<Boid>(boid));
                    boid.currentChunk = std::make_optional(std::make_shared<Chunk>(chunk));
                    // Continue to next boid to avoid the same boid being owned by multiple chunks
                    goto next;
                }
            }
        }
    next:
        continue;
    }
}

int ChunkedFlock::countBoids() {
    // Debug function to make sure no boids are owned by more than one chunk
    int count = 0;

    for (std::vector<Chunk>& row : this->chunks) {
        for (Chunk& chunk : row) {
            count += chunk.owned.size();
        }
    }

    return count;
}

std::list<std::pair<int, int>> ChunkedFlock::generateOffsets(int xRadius, int yRadius) {
    std::list<std::pair<int, int>> offsets;

    // Iterate over a rectangular area around the chunk
    for (int i = -xRadius; i <= xRadius; ++i) {
        for (int j = -yRadius; j <= yRadius; ++j) {
            // Exclude the furthermost horizontal and vertical chunks
            if ((i == xRadius && j == 0) || (i == -xRadius && j == 0) || (i == 0 && j == yRadius) || (i == 0 && j == -yRadius)) {
                continue;
            }
            // Calculate the distance from the center
            double distance = sqrt((i * i * 1.0) / (xRadius * xRadius) + (j * j * 1.0) / (yRadius * yRadius));
            // If the distance is less than or equal to 1, include the offset
            if (distance <= 1.0) {
                offsets.push_back({ i, j });
            }
        }
    }

    return offsets;
}

std::list<std::unique_ptr<Chunk>> ChunkedFlock::getAdjacentChunks(std::pair<int, int> index, std::list<std::pair<int, int>> offsets) {
    std::list<std::unique_ptr<Chunk>> adjacent;

    for (std::pair<int, int> offset : offsets) {
        std::pair<int, int> adjacentIndex;

        adjacentIndex.first = (index.first + offset.first + this->chunks.size()) % this->chunks.size();
        adjacentIndex.second = (index.second + offset.second + this->chunks[0].size()) % this->chunks[0].size();

        adjacent.push_back(std::move(std::unique_ptr<Chunk>(&this->chunks[adjacentIndex.first][adjacentIndex.second])));
    }

    return adjacent;
}

void CPUFlock::look(int i, const sf::Vector2u& dimensions) {
    // Update i-th boid's visible list

    // Get visibility radius and chunk dimensions
    int visibilityRadius = std::ceil(this->boids[i].visibility * this->boids[i].radius);
    int chunkWidth = this->chunks[0][0].bottomRight.x - this->chunks[0][0].topLeft.x;
    int chunkHeight = this->chunks[0][0].bottomRight.y - this->chunks[0][0].topLeft.y;

    // Define the x and y chunk visibility radius and add 1 for for smoothing
    int chunkXRadius = (visibilityRadius / chunkWidth) + (visibilityRadius % chunkWidth != 0) + 1;
    int chunkYRadius = (visibilityRadius / chunkHeight) + (visibilityRadius % chunkHeight != 0) + 1;

    std::list<std::unique_ptr<Chunk>> adjacent;
    std::list<std::pair<int, int>> offsets = this->generateOffsets(chunkXRadius, chunkYRadius);

    if (this->boids[i].currentChunk) {
        adjacent = this->getAdjacentChunks(this->boids[i].currentChunk.value()->index, offsets);
    }
}