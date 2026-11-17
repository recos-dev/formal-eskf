// SPDX-License-Identifier: BSD-3-Clause
#include <iostream>
#include <iomanip>
#include <lib/geo/geo.h>

int main()
{
    std::cout << std::setprecision(17);
    double lat0, lon0, lat, lon;
    while (std::cin >> lat0 >> lon0 >> lat >> lon)
    {
        MapProjection projection(lat0, lon0);
        float north{}, east{};
        double back_lat{}, back_lon{};
        projection.project(lat, lon, north, east);
        projection.reproject(north, east, back_lat, back_lon);
        std::cout << north << ' ' << east << ' ' << back_lat << ' ' << back_lon << '\n';
    }
}
