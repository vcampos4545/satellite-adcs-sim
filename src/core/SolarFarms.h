#pragma once
#include <glm/glm.hpp>
#include <array>

// A solar farm's real-world site: name plus geodetic lat/lon (degrees) --
// same spherical-Earth convention GroundStation uses (see its own header
// comment). Reflect-mode target candidates for a Reflect-Orbital-style
// mission: data + globe visualization only for now (see Simulation's
// showSolarFarms toggle) -- REFLECT still points at adcs.target as today;
// auto-selecting/tracking a solar farm the way ground stations are tracked
// for TARGET modes is a natural follow-up, not implemented here.
struct SolarFarm
{
  const char *name;
  double latDeg;
  double lonDeg;
};

// ~50 real, geographically-distributed utility-scale solar farms across
// all 5 continents (sourced during this feature's research) -- reduced
// from many real candidates to a simulation-reasonable number, the same
// "small fixed compiled list, no data-file parser needed" reasoning
// GROUND_STATIONS uses. Coordinates are site-center-level precision (like
// GROUND_STATIONS' own city-level coordinates), not survey-grade.
inline constexpr std::array<SolarFarm, 50> SOLAR_FARMS = {{
    // Asia
    {"Bhadla Solar Park, India", 27.5397, 71.9153},
    {"Pavagada Solar Park, India", 14.25, 77.45},
    {"Kurnool Ultra Mega Solar Park, India", 15.75, 78.35},
    {"Kamuthi Solar Power Project, India", 9.36, 78.38},
    {"Tengger Desert Solar Park, China", 37.5620, 105.0413},
    {"Longyangxia Dam Solar Park, China", 36.15, 100.59},
    {"Datong Solar Power Top Runner Base, China", 40.09, 113.30},
    {"Golmud Solar Park, China", 36.40, 94.90},
    {"Al Dhafra Solar Park, UAE", 23.87, 54.02},
    {"Mohammed bin Rashid Al Maktoum Solar Park, UAE", 24.76, 55.45},
    {"Sakaka Solar Plant, Saudi Arabia", 29.97, 40.21},
    {"Quaid-e-Azam Solar Park, Pakistan", 28.90, 71.90},
    {"Dau Tieng Solar Power Complex, Vietnam", 11.35, 106.25},
    {"Kagoshima Nanatsujima Mega Solar Park, Japan", 31.59, 130.33},
    // Africa
    {"Benban Solar Park, Egypt", 24.4560, 32.7390},
    {"Noor Ouarzazate, Morocco", 31.0492, -6.8694},
    {"Kom Ombo Solar Plant, Egypt", 24.45, 32.95},
    {"Jasper Solar Energy Project, South Africa", -29.50, 24.40},
    {"De Aar Solar Power Plant, South Africa", -30.65, 24.00},
    {"Garissa Solar Power Plant, Kenya", -0.45, 39.65},
    {"Nzema Solar Park, Ghana", 5.10, -2.50},
    // Europe
    {"Nunez de Balboa, Spain", 38.4528, -6.2257},
    {"Francisco Pizarro Solar Plant, Spain", 39.20, -6.30},
    {"Cestas Solar Park, France", 44.70, -0.70},
    {"Solarpark Meuro, Germany", 51.50, 13.90},
    {"Solarpark Weesow-Willmersdorf, Germany", 52.70, 13.70},
    {"Templin Solar Park, Germany", 53.10, 13.50},
    {"Pokrovske Solar Power Station, Ukraine", 46.80, 34.50},
    {"Andasol Solar Power Station, Spain", 37.23, -3.07},
    // North America
    {"Topaz Solar Farm, USA", 35.3833, -120.0599},
    {"Solar Star, USA", 34.86, -119.79},
    {"Desert Sunlight Solar Farm, USA", 33.80, -115.40},
    {"Copper Mountain Solar Facility, USA", 35.80, -114.90},
    {"Mount Signal Solar, USA", 32.67, -115.60},
    {"Springbok Solar Farm, USA", 35.00, -119.60},
    {"Roadrunner Solar, USA", 32.40, -102.40},
    {"Villanueva Solar Park, Mexico", 25.5859, -103.0450},
    {"Travers Solar Project, Canada", 50.30, -112.20},
    // South America
    {"Cauchari Solar Park, Argentina", -23.85, -66.70},
    {"Finis Terrae Solar Plant, Chile", -24.00, -69.50},
    {"El Romero Solar, Chile", -29.35, -70.90},
    {"Rubi Solar Plant, Chile", -22.50, -69.50},
    {"Pirapora Solar Complex, Brazil", -17.35, -44.90},
    {"Sao Goncalo Solar Complex, Brazil", -7.29, -37.35},
    // Oceania
    {"Western Downs Green Power Hub, Australia", -26.955, 150.677},
    {"Darlington Point Solar Farm, Australia", -34.55, 146.00},
    {"Coleambally Solar Farm, Australia", -34.80, 145.90},
    {"Bungala Solar Farm, Australia", -32.60, 137.90},
    {"Limondale Solar Farm, Australia", -34.20, 142.90},
    {"Sunraysia Solar Farm, Australia", -34.35, 142.40},
}};

// A farm's current real ECI position, accounting for Earth's rotation --
// same ECEF->ECI rotation groundStationPositionEci uses.
glm::dvec3 solarFarmPositionEci(const SolarFarm &farm, double thetaGstRad);
