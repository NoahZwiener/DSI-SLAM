#ifndef OCCUPANCYGRID_H
#define OCCUPANCYGRID_H

#include <string>
#include "Atlas.h"

namespace ORB_SLAM3
{
class OccupancyGrid
{
public:
    enum Plane { PLANE_XZ=0, PLANE_XY=1 };

    // Save occupancy grid from the given atlas.
    // - basename: used for .pgm and .yaml files
    // - resolution: meters per pixel
    // - plane: projection plane (XZ or XY)
    // - minPoints: minimum number of MapPoints in a cell to consider it occupied
    // - dilation: radius in cells to dilate occupied pixels (thicken boundaries)
    static void SaveFromAtlas(Atlas* pAtlas, const std::string &basename, float resolution = 0.05f,
                              Plane plane = PLANE_XZ, int minPoints = 1, int dilation = 1);
};

}

#endif // OCCUPANCYGRID_H
