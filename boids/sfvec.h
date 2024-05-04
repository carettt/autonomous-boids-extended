#pragma once

#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <math.h>
#include <list>
#include <iostream>

#include <random>
#include <vector>
#include <chrono>
#include <thread>

#include <CL/sycl.hpp>

// Vector math functions for SFML vectors
namespace sfvec {

    // Global constant to convert from radians to degrees
    const float TO_DEGREES = 180.f / M_PI;

    // Static constants for zero vectors
    static const sf::Vector2f ZEROF = sf::Vector2f(0.f, 0.f);
    static const sf::Vector2i ZEROI = sf::Vector2i(0, 0);
    static const sf::Vector2u ZEROU = sf::Vector2u(0, 0);

    // Print vector for debug
    template<typename T>
    void println(const sf::Vector2<T>& v) {
        std::cout << "{" << v.x << ", " << v.y << "}" << std::endl;
    }

    template<typename T>
    int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    // Return dot product of two vectors
    template<typename T>
    float dot(sf::Vector2<T> a, sf::Vector2<T> b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    // Get distance between two vectors
    template<typename T>
    float getDistance(const sf::Vector2<T>& a, const sf::Vector2<T>& b) {
        return sqrt(pow((b.x - a.x), 2) + pow(b.y - a.y, 2));
    }

    // Get relative position of a vector in a toroidal space in relation to another vector
    template<typename T>
    sf::Vector2<T> getRelativeToroidalPosition(const sf::Vector2<T>& of, const sf::Vector2<T>& to, const sf::Vector2u& dimensions) {
        int m = -sgn(of.x - to.x);
        int n = -sgn(of.y - to.y);

        sf::Vector2<T> avatars[4];
        avatars[0] = of;
        avatars[1] = of + sf::Vector2<T>(m * dimensions.x, 0);
        avatars[2] = of + sf::Vector2<T>(0, n * dimensions.y);
        avatars[3] = of + sf::Vector2<T>(m * dimensions.x, n * dimensions.y);

        int closest = 0;

        for (int i = 1; i < 4; i++) {
            float distance = sfvec::getDistance(to, avatars[i]);
            float closestDistance = sfvec::getDistance(to, avatars[closest]);

            if (distance < closestDistance) {
                closest = i;
            }
        }

        return avatars[closest];
    }

    // Get distance between two vectors in a toroidal space
    template<typename T>
    float getToroidalDistance(const sf::Vector2<T>& a, const sf::Vector2<T>& b, const sf::Vector2u& dimensions) {
        // Get distances of vector components in each dimension
        float dx = abs(b.x - a.x);
        float dy = abs(b.y - a.y);

        // If the distance is greater than half the dimension's total length,
        // it is shorter to go the opposite direction, thus the real distance is the dimension - previous distance
        if (dx > (dimensions.x / 2)) {
            dx = dimensions.x - dx;
        }

        if (dy > (dimensions.y / 2)) {
            dy = dimensions.y - dy;
        }

        // Return true distance
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    // Get magnitude of vector
    template<typename T>
    float getMagnitude(const sf::Vector2<T>& v) {
        return sqrt(pow(v.x, 2) + pow(v.y, 2));
    }

    // Normalize vector
    template<typename T>
    sf::Vector2<T> normalize(const sf::Vector2<T>& v) {
        sf::Vector2<T> normalized = v / sfvec::getMagnitude(v);

        return normalized;
    }

    // Clamp vector magnitude to max
    template<typename T>
    sf::Vector2<T> clampMagnitude(const sf::Vector2<T>& v, T max) {
        return sfvec::getMagnitude(v) > max ? sfvec::normalize(v) * max : v;
    }

    // Get rotation of vector
    template<typename T>
    float getRotation(const sf::Vector2<T>& v) {
        // Handle v.x being 0, to avoid zero division
        if (v.x == 0) {
            if (v.y > 0) {
                return 90.f;
            }
            else if (v.y < 0) {
                return 270.f;
            }
            else {
                return NAN;
            }
        }
        else {
            return atan((-1 * v.y) / v.x) * TO_DEGREES;
        }
    }

    template<typename T>
    T clamp(T val, T max) {
        return val > max ? max : val;
    }
}