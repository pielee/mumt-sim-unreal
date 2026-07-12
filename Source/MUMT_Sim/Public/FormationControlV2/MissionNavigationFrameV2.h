#pragma once

#include <array>
#include <cstdint>

namespace FormationControlV2 {

struct Vec3dV2 {
    double X{}, Y{}, Z{};
    Vec3dV2 operator+(const Vec3dV2 &o) const { return {X + o.X, Y + o.Y, Z + o.Z}; }
    Vec3dV2 operator-(const Vec3dV2 &o) const { return {X - o.X, Y - o.Y, Z - o.Z}; }
    bool IsFinite() const;
};

struct MissionOriginConfigV2 {
    double GeodeticLatitudeRad{};
    double LongitudeRad{};
    double EllipsoidHeightM{};
    std::uint32_t OriginGeneration{};
    bool bValid{};
};

struct MissionNedResultV2 {
    Vec3dV2 Ned{};
    std::uint32_t OriginGeneration{};
    bool bValid{};
};

class MissionNavigationFrameV2 {
public:
    bool SetOrigin(const MissionOriginConfigV2 &config);
    bool IsValid() const { return bValid; }
    std::uint32_t GetOriginGeneration() const { return OriginGeneration; }
    Vec3dV2 GetOriginEcefM() const { return OriginEcefM; }

    static Vec3dV2 GeodeticToEcefM(double geodeticLatitudeRad, double longitudeRad,
                                   double ellipsoidHeightM, bool &valid);
    MissionNedResultV2 EcefPositionToMissionNedM(const Vec3dV2 &ecefPositionM) const;
    MissionNedResultV2 EcefVelocityToMissionNedMps(const Vec3dV2 &ecefVelocityMps) const;
    Vec3dV2 MissionNedPositionToEcefM(const Vec3dV2 &nedPositionM, bool &valid) const;
    Vec3dV2 MissionNedVelocityToEcefMps(const Vec3dV2 &nedVelocityMps, bool &valid) const;

private:
    Vec3dV2 RotateEcefToNed(const Vec3dV2 &ecefVector) const;
    Vec3dV2 RotateNedToEcef(const Vec3dV2 &nedVector) const;

    Vec3dV2 OriginEcefM{};
    std::array<std::array<double, 3>, 3> EcefToNed{};
    std::uint32_t OriginGeneration{};
    bool bValid{};
};

} // namespace FormationControlV2
