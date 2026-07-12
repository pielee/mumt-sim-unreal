#include "FormationControlV2/MissionNavigationFrameV2.h"

#include <cmath>

namespace FormationControlV2 {
namespace {
constexpr double Wgs84A = 6378137.0;
constexpr double Wgs84InverseFlattening = 298.257223563;
constexpr double kMissionNavigationPi = 3.1415926535897932384626433832795;
bool IsFiniteMissionNavigation(double value) { return std::isfinite(value); }
}

bool Vec3dV2::IsFinite() const { return IsFiniteMissionNavigation(X) && IsFiniteMissionNavigation(Y) && IsFiniteMissionNavigation(Z); }

Vec3dV2 MissionNavigationFrameV2::GeodeticToEcefM(
    double latitude, double longitude, double height, bool &valid)
{
    valid = false;
    if (!IsFiniteMissionNavigation(latitude) || !IsFiniteMissionNavigation(longitude) || !IsFiniteMissionNavigation(height)
        || latitude < -0.5 * kMissionNavigationPi || latitude > 0.5 * kMissionNavigationPi) return {};
    const double flattening = 1.0 / Wgs84InverseFlattening;
    const double eccentricitySquared = flattening * (2.0 - flattening);
    const double sinLat = std::sin(latitude), cosLat = std::cos(latitude);
    const double sinLon = std::sin(longitude), cosLon = std::cos(longitude);
    const double primeVertical = Wgs84A / std::sqrt(1.0 - eccentricitySquared * sinLat * sinLat);
    Vec3dV2 result{
        (primeVertical + height) * cosLat * cosLon,
        (primeVertical + height) * cosLat * sinLon,
        (primeVertical * (1.0 - eccentricitySquared) + height) * sinLat};
    valid = result.IsFinite();
    return valid ? result : Vec3dV2{};
}

bool MissionNavigationFrameV2::SetOrigin(const MissionOriginConfigV2 &config)
{
    *this = MissionNavigationFrameV2{};
    if (!config.bValid) return false;
    bool originValid = false;
    const Vec3dV2 origin = GeodeticToEcefM(config.GeodeticLatitudeRad, config.LongitudeRad,
                                           config.EllipsoidHeightM, originValid);
    if (!originValid) return false;
    const double sinLat = std::sin(config.GeodeticLatitudeRad);
    const double cosLat = std::cos(config.GeodeticLatitudeRad);
    const double sinLon = std::sin(config.LongitudeRad);
    const double cosLon = std::cos(config.LongitudeRad);
    EcefToNed = {{{-sinLat * cosLon, -sinLat * sinLon, cosLat},
                  {-sinLon, cosLon, 0.0},
                  {-cosLat * cosLon, -cosLat * sinLon, -sinLat}}};
    OriginEcefM = origin;
    OriginGeneration = config.OriginGeneration;
    bValid = true;
    return true;
}

Vec3dV2 MissionNavigationFrameV2::RotateEcefToNed(const Vec3dV2 &v) const
{
    return {EcefToNed[0][0] * v.X + EcefToNed[0][1] * v.Y + EcefToNed[0][2] * v.Z,
            EcefToNed[1][0] * v.X + EcefToNed[1][1] * v.Y + EcefToNed[1][2] * v.Z,
            EcefToNed[2][0] * v.X + EcefToNed[2][1] * v.Y + EcefToNed[2][2] * v.Z};
}

Vec3dV2 MissionNavigationFrameV2::RotateNedToEcef(const Vec3dV2 &v) const
{
    return {EcefToNed[0][0] * v.X + EcefToNed[1][0] * v.Y + EcefToNed[2][0] * v.Z,
            EcefToNed[0][1] * v.X + EcefToNed[1][1] * v.Y + EcefToNed[2][1] * v.Z,
            EcefToNed[0][2] * v.X + EcefToNed[1][2] * v.Y + EcefToNed[2][2] * v.Z};
}

MissionNedResultV2 MissionNavigationFrameV2::EcefPositionToMissionNedM(const Vec3dV2 &ecef) const
{
    if (!bValid || !ecef.IsFinite()) return {};
    const Vec3dV2 ned = RotateEcefToNed(ecef - OriginEcefM);
    return ned.IsFinite() ? MissionNedResultV2{ned, OriginGeneration, true} : MissionNedResultV2{};
}

MissionNedResultV2 MissionNavigationFrameV2::EcefVelocityToMissionNedMps(const Vec3dV2 &ecef) const
{
    if (!bValid || !ecef.IsFinite()) return {};
    const Vec3dV2 ned = RotateEcefToNed(ecef);
    return ned.IsFinite() ? MissionNedResultV2{ned, OriginGeneration, true} : MissionNedResultV2{};
}

Vec3dV2 MissionNavigationFrameV2::MissionNedPositionToEcefM(const Vec3dV2 &ned, bool &valid) const
{
    valid = bValid && ned.IsFinite();
    if (!valid) return {};
    const Vec3dV2 result = OriginEcefM + RotateNedToEcef(ned);
    valid = result.IsFinite();
    return valid ? result : Vec3dV2{};
}

Vec3dV2 MissionNavigationFrameV2::MissionNedVelocityToEcefMps(const Vec3dV2 &ned, bool &valid) const
{
    valid = bValid && ned.IsFinite();
    if (!valid) return {};
    const Vec3dV2 result = RotateNedToEcef(ned);
    valid = result.IsFinite();
    return valid ? result : Vec3dV2{};
}

} // namespace FormationControlV2
