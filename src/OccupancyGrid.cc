#include "OccupancyGrid.h"
#include "MapPoint.h"
#include <fstream>
#include <vector>
#include <limits>
#include <cmath>

namespace ORB_SLAM3
{

void OccupancyGrid::SaveFromAtlas(Atlas* pAtlas, const std::string &basename, float resolution, Plane plane, int minPoints, int dilation)
{
    if(!pAtlas)
        return;

    std::vector<MapPoint*> vpMPs = pAtlas->GetAllMapPoints();
    if(vpMPs.empty())
        return;

    float amin1 = std::numeric_limits<float>::max();
    float amin2 = std::numeric_limits<float>::max();
    float amax1 = -std::numeric_limits<float>::max();
    float amax2 = -std::numeric_limits<float>::max();

    for(auto pMP : vpMPs)
    {
        if(!pMP || pMP->isBad()) continue;
        Eigen::Vector3f pos = pMP->GetWorldPos();
        float a1 = (plane==PLANE_XZ) ? pos(0) : pos(0); // x
        float a2 = (plane==PLANE_XZ) ? pos(2) : pos(1); // z or y
        if(a1 < amin1) amin1 = a1;
        if(a1 > amax1) amax1 = a1;
        if(a2 < amin2) amin2 = a2;
        if(a2 > amax2) amax2 = a2;
    }

    if(amin1>amax1 || amin2>amax2) return;

    // Add small margin
    float margin = resolution * 10;
    amin1 -= margin; amin2 -= margin; amax1 += margin; amax2 += margin;

    int width = std::max(1, static_cast<int>(std::ceil((amax1 - amin1) / resolution)));
    int height = std::max(1, static_cast<int>(std::ceil((amax2 - amin2) / resolution)));

    // Limit maximum size to avoid huge maps
    const int MAX_DIM = 10000;
    if(width > MAX_DIM || height > MAX_DIM)
    {
        // avoid creating enormous maps
        return;
    }

    // Count points per cell
    std::vector<int> counts(width * height, 0);
    for(auto pMP : vpMPs)
    {
        if(!pMP || pMP->isBad()) continue;
        Eigen::Vector3f pos = pMP->GetWorldPos();
        float a1 = (plane==PLANE_XZ) ? pos(0) : pos(0);
        float a2 = (plane==PLANE_XZ) ? pos(2) : pos(1);
        int col = static_cast<int>(std::floor((a1 - amin1) / resolution));
        int row = static_cast<int>(std::floor((a2 - amin2) / resolution));
        if(col < 0 || col >= width || row < 0 || row >= height) continue;
        int rimg = height - 1 - row;
        counts[rimg * width + col]++;
    }

    // Threshold to create binary occupied map
    std::vector<unsigned char> grid(width * height, 205); // unknown
    std::vector<unsigned char> occ(width * height, 0);
    for(int i=0; i<width*height; ++i){
        if(counts[i] >= minPoints) occ[i] = 1;
    }

    // Apply dilation to thicken boundaries
    if(dilation > 0){
        std::vector<unsigned char> occ2 = occ;
        for(int r=0; r<height; ++r){
            for(int c=0; c<width; ++c){
                if(occ[r*width + c]){
                    int r0 = std::max(0, r-dilation);
                    int r1 = std::min(height-1, r+dilation);
                    int c0 = std::max(0, c-dilation);
                    int c1 = std::min(width-1, c+dilation);
                    for(int rr=r0; rr<=r1; ++rr){
                        for(int cc=c0; cc<=c1; ++cc){
                            occ2[rr*width + cc] = 1;
                        }
                    }
                }
            }
        }
        occ.swap(occ2);
    }

    for(int i=0;i<width*height;++i){
        if(occ[i]) grid[i] = 0; // occupied
        else if(counts[i]==0) grid[i] = 205; // unknown
        else grid[i] = 254; // free (observed but below threshold)
    }

    // Save PGM
    std::string pgm = basename + ".pgm";
    std::ofstream ofs(pgm, std::ios::binary);
    if(!ofs.is_open()) return;
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(grid.data()), grid.size());
    ofs.close();

    // Save YAML metadata
    std::string yml = basename + ".yaml";
    std::ofstream yf(yml);
    if(!yf.is_open()) return;
    yf << "image: " << pgm << "\n";
    yf << "resolution: " << resolution << "\n";
    // origin: [x_min, y_min, yaw]
    float originX = amin1;
    float originY = amin2; // if PLANE_XZ this corresponds to world Z; if PLANE_XY corresponds to world Y
    yf << "origin: [" << originX << ", " << originY << ", 0.0]" << "\n";
    yf << "negate: 0\n";
    yf << "occupied_thresh: 0.65\n";
    yf << "free_thresh: 0.196\n";
    yf.close();
}

} // namespace
