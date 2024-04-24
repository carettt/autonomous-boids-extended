#include "boid.h"
#include "chunk.h"

template<typename F>
flocks::ChunkedFlock::ChunkedFlock(F dna, float sWeight, float cWeight, float aWeight, const int& splits, const sf::Vector2u& dimensions) :
    Flock(dna, sWeight, cWeight, aWeight)
{
    // Initialize chunks vector
    chunks = std::vector<std::vector<Chunk>>(splits, std::vector<Chunk>(splits, Chunk()));

    // Divide window area into splits^2 chunks
    for (int i = 0; i < splits; i++) {
        for (int j = 0; j < splits; j++) {
            sf::Vector2f topLeft((dimensions.x / splits) * i, (dimensions.y / splits) * j);
            sf::Vector2f bottomRight((dimensions.x / splits) * (i + 1), (dimensions.y / splits) * (j + 1));
            chunks[i][j] = Chunk(topLeft, bottomRight);
            chunks[i][j].position = sf::Vector2u(i, j);
        }
    }
}

void flocks::ChunkedFlock::localizeBoids() {
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
                    chunk.owned.push_back(std::shared_ptr<Boid>(&boid));
                    boid.currentChunk = std::shared_ptr<Chunk>(&chunk);
                    // Continue to next boid to avoid the same boid being owned by multiple chunks
                    goto next;
                }
            }
        }
    next:
        continue;
    }
}

int flocks::ChunkedFlock::countBoids() {
    // Debug function to make sure no boids are owned by more than one chunk
    int count = 0;

    for (std::vector<Chunk>& row : this->chunks) {
        for (Chunk& chunk : row) {
            count += chunk.owned.size();
        }
    }

    return count;
}

std::list<std::pair<int, int>> flocks::ChunkedFlock::generateOffsets(int xRadius, int yRadius) {
    std::list<std::pair<int, int>> offsets;

    // Iterate over a rectangular area around the chunk
    for (int i = -xRadius; i <= xRadius; ++i) {
        for (int j = -yRadius; j <= yRadius; ++j) {
            // Exclude the furthermost horizontal and vertical chunks (and the chunk at (0, 0)
            if ((i == xRadius && j == 0) || (i == -xRadius && j == 0) || (i == 0 && j == yRadius) || (i == 0 && j == -yRadius) || (i == 0 && j == 0)) {
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

std::list<std::unique_ptr<Chunk>> flocks::ChunkedFlock::getAdjacentChunks(std::pair<int, int> index, std::list<std::pair<int, int>> offsets) {
    std::list<std::unique_ptr<Chunk>> adjacent;

    for (std::pair<int, int> offset : offsets) {
        std::pair<int, int> adjacentIndex;

        adjacentIndex.first = (index.first + offset.first + this->chunks.size()) % this->chunks.size();
        adjacentIndex.second = (index.second + offset.second + this->chunks[0].size()) % this->chunks[0].size();

        adjacent.push_back(std::move(std::unique_ptr<Chunk>(&this->chunks[adjacentIndex.first][adjacentIndex.second])));
    }

    return adjacent;
}