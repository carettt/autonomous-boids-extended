#include "flocks.h"

void Flock::look(int i) {
    // Update i-th boid's visible list

    // Loop through all boids
    for (int j = 0; j < this->size; j++) {
        float relativeDistance = sfvec::getToroidalDistance(this->boids[i].position, this->boids[j].position, this->window->getSize());

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

void Flock::update(float deltaTime) {
    // Call all necessary frametime functions on all boids

    if (deltaTime) {
        // Loop through all boids
        for (int i = 0; i < this->size; i++) {
            // Update i-th boid's visible list
            this->look(i);
            // Update i-th boid's forces
            this->boids[i].update(this->window->getSize(), this->w, this->gen);
            // Add boid to render queue
            this->boids[i].draw(this->window, deltaTime);
            // Clear i-th boid's visible list
            this->forget(i);
        }
    }
}

void ChunkedFlock::localizeBoids() {
    // Split boids into their respective chunks

    // Clear all chunks owned lists
    for (std::vector<Chunk>& row : this->chunks) {
        for (Chunk& chunk : row) {
            chunk.owned.clear();
        }
    }

    //Loop through all boids
    for (int i = 0; i < this->size; i++) {
        // Loop through all chunks
        for (std::vector<Chunk>& row : this->chunks) {
            for (Chunk& chunk : row) {

                // If boid is within chunk bounds then the chunk 'owns' the boid
                if (this->boids[i].position.x >= chunk.topLeft.x &&
                    this->boids[i].position.x <= chunk.bottomRight.x &&
                    this->boids[i].position.y >= chunk.topLeft.y &&
                    this->boids[i].position.y <= chunk.bottomRight.y)
                {
                    chunk.owned.emplace_back(this->boids[i]);
                    this->boids[i].currentChunk = std::make_optional(std::make_shared<Chunk>(chunk));
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

        adjacent.push_back(std::move(std::make_unique<Chunk>(this->chunks[adjacentIndex.first][adjacentIndex.second])));
    }

    return adjacent;
}

void CPUFlock::update(float deltaTime) {
    // Call all necessary frametime functions on all boids
    std::cout << "new frame\n";

    if (deltaTime) {
        std::cout << "deltaTime: " << deltaTime << "\n";

        this->updateSync.arrive_and_wait();

        for (int i = 0; i < this->size; i++) {
            this->boids[i].draw(this->window, deltaTime);
        }
    }

    this->localizeBoids();
    this->lookSync.arrive_and_wait();
}