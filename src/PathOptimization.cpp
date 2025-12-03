/*
 * PathOptimization.cpp
 * FIXED: Proper distance summation and memory usage
 */

#include "PathOptimization.h"

PathOptimization::PathOptimization() {}

void PathOptimization::optimize(String &path, long segments[], int &pathLength) {
    // Build new path using temporary small buffer
    String newPath = "";
    newPath.reserve(pathLength + 1); // Pre-allocate
    
    long newSegments[100]; // Match main.cpp array size
    int newIndex = 0;
    
    int i = 0;
    while (i < pathLength) {
        // Check for 3-character optimization patterns
        if (i <= pathLength - 3) {
            String sub = path.substring(i, i + 3);
            
            // Sum the distances of the 3 segments being combined
            long combinedDist = segments[i] + segments[i+1] + segments[i+2];
            
            if (sub == "LBR") {
                // Left + Back + Right = U-turn (Back)
                newPath += 'B';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "LBS") {
                // Left + Back + Straight = Right turn
                newPath += 'R';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "RBL") {
                // Right + Back + Left = U-turn (Back)
                newPath += 'B';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "SBL") {
                // Straight + Back + Left = Right turn
                newPath += 'R';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "SBS") {
                // Straight + Back + Straight = U-turn
                newPath += 'B';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "LBL") {
                // Left + Back + Left = Straight (180° + 180° cancel out)
                newPath += 'S';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "RBR") {
                // Right + Back + Right = Straight
                newPath += 'S';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "SBR") {
                // Straight + Back + Right = Left
                newPath += 'L';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "RBS") {
                // Right + Back + Straight = Left
                newPath += 'L';
                newSegments[newIndex] = combinedDist;
                i += 3;
                newIndex++;
                continue;
            }
        }
        
        // No optimization found, copy current segment
        newPath += path[i];
        newSegments[newIndex] = segments[i];
        i++;
        newIndex++;
    }
    
    // Copy back optimized path
    path = newPath;
    pathLength = newIndex;
    
    // Copy segment distances back
    for(int k = 0; k < newIndex && k < 100; k++) {
        segments[k] = newSegments[k];
    }
}