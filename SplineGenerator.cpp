#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std;
#include "SplineHeader.h"
int main() {
    vector<sPoint2D> coords = pointReader("Coordinates.txt");
    vector<sPoint2D> interCoords, gradientCoords, outerCoords;
    sSpline splineGen;
    splineGen.points = coords;
    float stepSize = 0.1, fTrackWidth = 10;
    sPoint2D interCoord, gradientCoord, gradientCoordUnNormd;
    for (float i = 0;i < coords.size(); i += stepSize) {
        if (i >= 35.6) {
            cout << i << endl;
        }
        interCoord = splineGen.GetSplinePoint(i);
        gradientCoord = splineGen.GetSplineGradient(i);
        float glen = sqrtf(gradientCoord.x * gradientCoord.x + gradientCoord.y * gradientCoord.y);
        outerCoords.push_back({ interCoord.x - fTrackWidth * (-gradientCoord.y / glen), interCoord.y - fTrackWidth * (gradientCoord.x / glen) });
        interCoords.push_back(interCoord);
        //cout << sqrt(pow(gradientCoord.x, 2) + pow(gradientCoord.y, 2)) << endl;
    }
    outPoints("interpolatedPoints.txt", interCoords);
    outPoints("outerCoords.txt", outerCoords);
    return 0;
}