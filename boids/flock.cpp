#include "flock.h"

// Sequential

template<typename F>
flocks::Flock::Flock(F dna, float sWeight, float cWeight, float aWeight) : w(sWeight, cWeight, aWeight) {
    // Loop through empty array
    for (int i = 0; i < this->size; i++) {
        // Assign i-th element to result of 'DNA' callback function, passing the index as an argument
        this->boids[i] = dna(i);
        this->boids[i].defaultTopSpeed = this->boids[i].topSpeed;
    }
}

void flocks::Flock::look(int i, const sf::Vector2u& dimensions) {
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

void flocks::Flock::forget(int index) {
    // Clear all boids' visible lists
    this->boids[index].visible.clear();
}

void flocks::Flock::update(std::shared_ptr<sf::RenderWindow> window, std::mt19937& gen, float deltaTime) {
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