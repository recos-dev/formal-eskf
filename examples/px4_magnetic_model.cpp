/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Host-side batch bridge to an externally supplied magnetic-model library.
#include <geo_mag_declination.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

int main()
{
    double latitude{}, longitude{};
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line))
    {
        std::istringstream row(line);
        std::string extra;
        if (!(row >> latitude >> longitude) || (row >> extra) || !std::isfinite(latitude) ||
            !std::isfinite(longitude) || latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0)
        {
            std::cerr << "error: expected finite latitude/longitude degrees in [-90,90] / [-180,180]\n";
            return 1;
        }
        auto const lat = static_cast<float>(latitude);
        auto const lon = static_cast<float>(longitude);
        double const declination = get_mag_declination_degrees(lat, lon);
        double const inclination = get_mag_inclination_degrees(lat, lon);
        double const strength = static_cast<double>(get_mag_strength_gauss(lat, lon)) * 100.0;
        if (!std::isfinite(declination) || !std::isfinite(inclination) || !std::isfinite(strength) || strength <= 0.0)
        {
            std::cerr << "error: magnetic model returned an invalid field\n";
            return 1;
        }
        std::cout << declination << ' ' << inclination << ' ' << strength << '\n';
    }
    std::cout.flush();
    if (std::cin.bad() || !std::cin.eof() || !std::cout)
    {
        std::cerr << "error: magnetic model stream failed\n";
        return 1;
    }
    return 0;
}
