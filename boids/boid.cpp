#include "boid.h"

bool leaderExists = false; // Global shared state

Boid::Boid() : visibility(5.f), topSpeed(25.f), position(0, 0), velocity(0, 0), radius(5) {
    this->triangle = sf::CircleShape(radius, 3);
    this->triangle.setPosition(this->position);
    this->triangle.setFillColor(sf::Color::White);
}

Boid::Boid(float x, float y, float radius, float topSpeed, sf::Vector2f v, float visibility) :
    visibility(visibility), topSpeed(topSpeed), position(x, y), velocity(v), radius(radius)
{
    this->triangle = sf::CircleShape(radius, 3);
    this->triangle.setOrigin(radius, radius);
    this->triangle.setPosition(this->position);
    this->triangle.setFillColor(sf::Color::White);
}

float Boid::escapeAcceleration(float t) {
    // Get escape velocity based on acceleration curve: https://www.desmos.com/calculator/pd0gtqrvbw

    return -pow(tanh((2.25f * pow(t, -0.4f)) - 3.f), 2) + 2.f;
}

void Boid::calculateSeparation(const sf::Vector2u& dimensions) {
    // Calculate separation force

    sf::Vector2f direction;
    this->separation = sfvec::ZEROF; // Reset separation force from last frame

    // Loop through visible boids
    for (const Boid* other : this->visible) {
        // Get relative position of other boid
        sf::Vector2f relativePosition = sfvec::getRelativeToroidalPosition(this->position, other->position, dimensions);
        float distance = sfvec::getToroidalDistance(this->position, other->position, dimensions);

        // Calculate direction vector from self to other boid
        direction = sfvec::normalize(this->position - other->position);

        // Normalize the direction vector to set the separation force's direction
        // The magnitude will be set by the amount of visible boids * separation weight
        if (!other->leader) {
            this->separation += (direction / (distance * distance));
        }
    }

    this->separation *= (float)this->visible.size();
}

void Boid::calculateCohesion(const sf::Vector2u& dimensions) {
    // Calculate cohesion force

    this->cohesion = sfvec::ZEROF; // Reset cohesion force from last frame

    // Update cohesion only if there are boids visible, to prevent incorrect calculation
    if (this->visible.size() > 0) {
        sf::Vector2f centre = sfvec::ZEROF;
        bool leaderVisible = false;

        // Loop through visible boids
        for (Boid* other : this->visible) {
            // Get relative position of other boid
            sf::Vector2f relativePosition = sfvec::getRelativeToroidalPosition(other->position, this->position, dimensions);

            // Calculate centre as average position of visible boids
            centre += relativePosition / (float)this->visible.size();

            if (other->leader) {
                centre = relativePosition;
                break;
            }
        }

        // Set cohesion to the displacement from the centre to self
        // This value is divided by the number of boids to ensure consistent cohesion despite density of flock
        this->cohesion = sfvec::normalize(centre - this->position) / (float)this->visible.size();
    }
}

void Boid::calculateAlignment() {
    // Calculate alignment force

    this->alignment = sfvec::ZEROF; // Reset alignment force from last frame

    // Update alignment only if there are boids visible, to prevent incorrect calculation
    if (this->visible.size() > 0) {
        // Loop through visible boids
        for (Boid* other : this->visible) {
            sf::Vector2f force = other->velocity / (float)this->visible.size();

            if (this->leader) {
                if (sfvec::dot(this->velocity, other->velocity) > 0) {
                    this->alignment += force * -0.6f;
                }
            }

            if (!other->leader) {
                // Calculate alignment as the average velocity of visible boids
                this->alignment += force;
            }
            else {
                this->alignment = other->velocity;
                break;
            }
        }
    }
}

void Boid::calculateEccentricity(const sf::Vector2u& dimensions) {
    // Calculate eccentricity of boid using Felipe Takaoka's eccentricity formula

    this->eccentricity = 0.f; // Reset eccentricity from last frame

    float t = this->radius + (((this->visibility * this->radius) - this->radius) / 2.f); // Midpoint distance between boid and visibility radius
    float sigma = ((this->visibility * this->radius) - this->radius) * 8.f; // Tuning value for eccentricity formula

    // Only attempt escape if boids visible (in a flock)
    if (this->visible.size() > 0) {
        sf::Vector2f centre = sfvec::ZEROF;

        // Loop through every visible boid
        for (Boid* other : this->visible) {
            sf::Vector2f relativePosition = sfvec::getRelativeToroidalPosition(this->position, other->position, dimensions);

            // Calculate centre of visible boid density as average position of visible boids
            centre += relativePosition / ((float)this->visible.size());
        }

        // Calculate eccentricity with a Gaussian-kernel-like distribution
        // If boid is surrounded by other boids, then eccentricity ~= 0
        // Values of eccentricity closer to 1 indicate the boid is near the edge of the flock
        this->eccentricity = exp(-pow(sfvec::getMagnitude(centre) - t, 2) / (2 * pow(sigma, 2)));
    }
}

void Boid::attemptEscape(std::mt19937& gen, sf::Vector2u dimensions) {
    // Attempt to escape flock with a random chance

    // Initialize random distribution
    std::uniform_real_distribution<float> rand(0.85f, 1.f);

    // Get time elapsed
    this->leaderTimerStop = std::chrono::high_resolution_clock::now();
    float timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(this->leaderTimerStop - this->leaderTimerStart).count();

    if (!this->leader) {
        // Update cohesion only if there are boids visible, to prevent incorrect calculation
        if (this->visible.size() > 0) {
            sf::Vector2f centre = sfvec::ZEROF;

            // Loop through visible boids
            for (Boid* other : this->visible) {
                // Get relative position of other boid
                sf::Vector2f relativePosition = sfvec::getRelativeToroidalPosition(other->position, this->position, dimensions);

                // Calculate centre as average position of visible boids
                centre += relativePosition / (float)this->visible.size();
            }

            // Calculate front back axis as the negative dot product of the normalized direction towards the centroid and the normalized velocity
            // Values closer to -1 indicate the boid is nearer to the back of the flock
            // Values closer to 1 indicate the boid is nearer to the front of the flock
            float frontBackAxis = sfvec::dot(sfvec::normalize(this->position - centre), sfvec::normalize(this->velocity));


            // Uncomment following lines to display high escape chances for debug
            //if (this->eccentricity * frontBackAxis > 0.7f) {
            //    std::cout << "------\n HIGH ESCAPE CHANCE !" << std::endl;

            //    std::cout << "eccentricity: " << this->eccentricity << std::endl;
            //    std::cout << "frontBackAxis: " << frontBackAxis << std::endl;
            //    std::cout << "combined: " << this->eccentricity * frontBackAxis << std::endl;

            //    std::cout << "------" << std::endl;
            //}

            // Calculate chance of escaping as eccentricity multiplied by the frotn back axis
            // If the front back axis is negative, it will never be greater than the random number, thus the boid wont escape if at the back of the flock
            if (frontBackAxis * this->eccentricity > rand(gen) && !leaderExists) {
                std::cout << "ESCAPING !" << std::endl;

                leaderExists = true;

                // Set boid as leader
                this->leader = true;

                // Adjust boid properties to reflect an escaping boid
                this->visibility *= 1.5f;
                this->triangle.setFillColor(sf::Color::Red);

                // Start leader timer
                this->leaderTimerStart = std::chrono::high_resolution_clock::now();
            }
        }
    }
    else {
        // Adjust top speed based on acceleration curve
        this->topSpeed = this->defaultTopSpeed * this->escapeAcceleration(timeElapsed / 1000);

        // If boid has been leader for longer than leaderDuration, reset boid
        if (timeElapsed > this->leaderDuration) {
            this->leader = false;
            this->topSpeed = this->defaultTopSpeed;
            this->visibility /= 1.5f;
            this->triangle.setFillColor(sf::Color::White);

            leaderExists = false;
        }
    }
}

void Boid::update(const sf::Vector2u& dimensions, Weights w, std::mt19937& gen) {
    // Update boid

    // Update position variable to this frame's
    this->position = this->triangle.getPosition();

    // Calculate forces
    this->calculateSeparation(dimensions);
    this->calculateCohesion(dimensions);
    this->calculateAlignment();

    // Leadership
    this->calculateEccentricity(dimensions);
    this->attemptEscape(gen, dimensions);

    // Uncomment following lines to print raw steering forces (no weights) for debugging
    //std::cout << "separation: ";
    //sfvec::println(this->separation);
    //std::cout << "cohesion: ";
    //sfvec::println(this->cohesion);
    //std::cout << "alignment: ";
    //sfvec::println(this->alignment);

    // Apply steering forces to velocity with weights
    this->velocity +=
        (this->separation * w.sWeight * (this->leader ? 1.f : 1.f)) + // Triple separation weight when escaping
        (this->cohesion * w.cWeight * (this->leader ? 1.f : 1.f)) +
        (this->alignment * w.aWeight * (this->leader ? 1.f : 1.f));

    // Uncomment next line to print velocity for debugging
    //sfvec::println(this->velocity);

    // Clamp velocity to top speed
    this->velocity = sfvec::clampMagnitude(this->velocity, this->topSpeed);
}

void Boid::draw(std::shared_ptr<sf::RenderWindow> window, double deltaTime) {
    // Draw boid triangle and handle rotation and looping around the screen

    float velocityHeading = sfvec::getRotation(this->velocity);

    sf::Vector2u canvasSize = window->getSize();
    sf::Vector2f newPosition;

    sf::CircleShape visibilitySphere(this->radius * this->visibility);

    // Update position
    this->triangle.setPosition(this->position + (this->velocity * (float) deltaTime));
    newPosition = triangle.getPosition();

    // Rotate triangle if |velocity| > 0
    if (!isnan(velocityHeading)) {
        this->triangle.setRotation(velocityHeading);
    }

    // Loop around when out of canvasSize
    if (this->position.x > canvasSize.x) {
        newPosition.x -= canvasSize.x;
    }
    else if (this->position.x < 0) {
        newPosition.x += canvasSize.x;
    }

    if (this->position.y > canvasSize.y) {
        newPosition.y -= canvasSize.y;
    }
    else if (this->position.y < 0) {
        newPosition.y += canvasSize.y;
    }

    // Update position
    this->triangle.setPosition(newPosition);

    // Draw visibility sphere
    visibilitySphere.setFillColor(sf::Color(0, 0, 150, 10));
    visibilitySphere.setOrigin(this->radius * this->visibility, this->radius * this->visibility);
    visibilitySphere.setPosition(newPosition);
    window->draw(visibilitySphere);

    // Add triangle to render queue
    window->draw(this->triangle);
}