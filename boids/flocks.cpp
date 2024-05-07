#include "flocks.h"

void Flock::look(int i) {
    // Update i-th boid's visible list

    // Loop through all boids
    for (int j = 0; j < this->size; j++) {
        float relativeDistance = sfvec::getToroidalDistance(this->boids[i].position, this->boids[j].position, this->window->getSize());

        // If distance between i-th boid and j-th boid is less than the visibility factor * radius of self, j-th boid is visible
        if (relativeDistance < (this->boids[i].radius * this->boids[i].visibility) && i != j) {
            this->boids[i].visible.push_back(&this->boids[j]);
        }
    }
}

void Flock::forget(int index) {
    // Clear all boids' visible lists
    this->boids[index].visible.clear();
}

void Flock::update(double deltaTime) {
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

void NaiveCPUFlock::boundedUpdate(int lower, int upper) {
    // Bounded update function adapted to work with multiple threads

    // Update this thread's boid's visible lists
    for (int i = lower; i < upper; i++) {
        this->look(i);
    }

    // Make sure all visible lists are updated to prevent data races (updating position while checking distance for visibility check)
    this->ready.arrive_and_wait();

    // Call rest of update functions
    for (int i = lower; i < upper; i++) {
        this->boids[i].update(this->window->getSize(), this->w, this->gen);
        this->forget(i);
    }
}

void NaiveCPUFlock::update(double deltaTime) {
    // Update function adapted to work with multiple threads

    // Run bounded update for every thread, splitting boids evenly(ish)
    for (int i = 0; i < this->flockThreads.size(); i++) {
        int sectionSize = this->size / this->flockThreads.size();

        int lower = sectionSize * i;
        int upper = (i == this->flockThreads.size() - 1) ? this->size : (sectionSize * (i + 1));

        this->flockThreads[i] = std::thread(&NaiveCPUFlock::boundedUpdate, this, lower, upper);
    }

    // Wait for updates
    for (std::thread& t : this->flockThreads) {
        t.join();
    }

    // Draw boids
    // Handled by main thread since OpenGL context can only be active in one thread at a time,
    // allowing threads to draw their own boids causes lots of mutex locking and significantly slows down execution
    for (Boid& boid : this->boids) {
        boid.draw(this->window, deltaTime);
    }
}

void ChunkedFlock::localizeBoids() {
    // Split boids into their respective chunks

    // Clear chunks from previous frame
    for (std::vector<Chunk>& row : this->chunks) {
        for (Chunk& chunk : row) {
            chunk.owned.clear();
        }
    }

    // Split boids into chunks using topLeft and bottomRight coordinates
    for (Boid& boid : this->boids) {
        for (std::vector<Chunk>& row : this->chunks) {
            for (Chunk& chunk : row) {
                if (boid.position.x >= chunk.topLeft.x && boid.position.x <= chunk.bottomRight.x &&
                    boid.position.y >= chunk.topLeft.y && boid.position.y <= chunk.bottomRight.y)
                {
                    chunk.owned.push_back(&boid);
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
    // Generate chunk offsets in an independent x and y radius in a elliptic(ish) area

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
    // Get list of adjacent chunks based on chunk offsets
    
    std::list<std::unique_ptr<Chunk>> adjacent;

    for (std::pair<int, int> offset : offsets) {
        std::pair<int, int> adjacentIndex;

        adjacentIndex.first = (static_cast<unsigned long long>(index.first) + offset.first + this->chunks.size()) % this->chunks.size();
        adjacentIndex.second = (static_cast<unsigned long long>(index.second) + offset.second + this->chunks[0].size()) % this->chunks[0].size();

        adjacent.push_back(std::move(std::make_unique<Chunk>(this->chunks[adjacentIndex.first][adjacentIndex.second])));
    }

    return adjacent;
}

void CPUFlock::update(double deltaTime) {
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
    std::cout << "0th boid's visible boids: " << this->boids[0].visible.size() << "\n";
}

void GPUFlock::update(double deltaTime) {
    // Update adapted to work with SYCL DPC++ kernel

    // Allocate USM pointers
    FlatBoid* sharedBoids = sycl::malloc_shared<FlatBoid>(this->size, this->q);
    unsigned int* dimensions = sycl::malloc_shared<unsigned int>(2, this->q);
    VisibleBoid* visible = sycl::malloc_shared<VisibleBoid>(pow(this->size, 2), this->q);
    unsigned int* flockSize = sycl::malloc_shared<unsigned int>(1, this->q);
    unsigned int* counter = sycl::malloc_shared<unsigned int>(1, this->q);

    // Populate USM pointers
    this->flattenBoids(sharedBoids);
    sf::Vector2u windowSize = this->window->getSize();
    dimensions[0] = windowSize.x;
    dimensions[1] = windowSize.y;
    *flockSize = this->size;
    *counter = 0;

    q.submit([&](sycl::handler& h) {
        h.parallel_for(sycl::range<2>(this->size, this->size), [=](sycl::id<2> idx) {

            // Get distances of vector components in each dimension
            float dx = sycl::abs(sharedBoids[idx[1]].x - sharedBoids[idx[0]].x);
            float dy = sycl::abs(sharedBoids[idx[1]].y - sharedBoids[idx[0]].y);

            float trueDistance;

            // If the distance is greater than half the dimension's total length,
            // it is shorter to go the opposite direction, thus the real distance is the dimension - previous distance
            if (dx > (dimensions[0] / 2)) {
                dx = dimensions[0] - dx;
            }

            if (dy > (dimensions[1] / 2)) {
                dy = dimensions[1] - dy;
            }

            // set distances array to true distance
            trueDistance = sycl::sqrt(sycl::pow(dx, (float)2) + sycl::pow(dy, (float)2));

            // Add to visible array if in visibility radius
            if (trueDistance < sharedBoids[idx[0]].visibilityRadius) {
                visible[*counter] = { sharedBoids[idx[1]].id, sharedBoids[idx[0]].id };
                *counter += 1;
            }
        });
    }).wait();

    // Loop through visible array and push boids to respective visible lists (excluding 'empty' elements and boids looking at themselves
    for (int i = 0; i < *counter; i++) {
        if (visible[i].lookingId != visible[i].visibleId) {
            this->boids[visible[i].lookingId].visible.push_back(&this->boids[visible[i].visibleId]);
        }
    }

    // Run rest of update functions
    for (int i = 0; i < this->size; i++) {
        this->boids[i].update(this->window->getSize(), this->w, this->gen);
        this->boids[i].draw(this->window, deltaTime);
        this->forget(i);
    } 

    // Free USM pointers
    sycl::free(sharedBoids, this->q);
    sycl::free(dimensions, this->q);
    sycl::free(visible, this->q);
    sycl::free(flockSize, this->q);
    sycl::free(counter, this->q);
}