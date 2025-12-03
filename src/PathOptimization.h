/*
 * PathOptimization.h
 * Implements the LSRB path simplification algorithm.
 * Uses Arduino String class for safety and simplicity.
 */

#pragma once

#include <Arduino.h> // Required for String class

class PathOptimization {
public:
    PathOptimization();

    // Optimizes the path string and its corresponding segment lengths array in-place.
    // pathLength is passed by reference and will be updated.
    void optimize(String &path, long segments[10000], int &pathLength);
};