#include "FormationControlV2/MissionNavigationFrameV2.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>

using namespace FormationControlV2;
namespace {
constexpr double Pi = 3.1415926535897932384626433832795;
constexpr double A = 6378137.0;
constexpr double F = 1.0 / 298.257223563;
constexpr double B = A * (1.0 - F);
int failures = 0, checks = 0;
double maxEcef = 0.0, maxPositionRoundTrip = 0.0, maxVelocityRoundTrip = 0.0;
double Norm(const Vec3dV2 &v) { return std::sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z); }
void Check(bool ok, const char *name) { ++checks; if (!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } }
bool Near(const Vec3dV2 &a, const Vec3dV2 &b, double t) { return Norm(a-b) <= t; }

Vec3dV2 Oracle(double lat, double lon, double h)
{
    const double e2 = F * (2.0 - F), s = std::sin(lat), c = std::cos(lat);
    const double n = A / std::sqrt(1.0 - e2*s*s);
    return {(n+h)*c*std::cos(lon), (n+h)*c*std::sin(lon), (n*(1.0-e2)+h)*s};
}
MissionNavigationFrameV2 Frame(double lat, double lon, double h, std::uint32_t gen=1)
{
    MissionNavigationFrameV2 f; Check(f.SetOrigin({lat,lon,h,gen,true}), "origin_setup"); return f;
}
}

int main()
{
    bool valid=false;
    Check(Near(MissionNavigationFrameV2::GeodeticToEcefM(0,0,0,valid), {A,0,0}, 1e-9) && valid, "equator_prime");
    Check(Near(MissionNavigationFrameV2::GeodeticToEcefM(0,Pi/2,0,valid), {0,A,0}, 1e-8) && valid, "east_90");
    Check(Near(MissionNavigationFrameV2::GeodeticToEcefM(Pi/2,0,0,valid), {0,0,B}, 1e-8) && valid, "north_pole");
    Check(Near(MissionNavigationFrameV2::GeodeticToEcefM(-Pi/2,0,1000,valid), {0,0,-(B+1000)}, 1e-8) && valid, "south_high");
    auto f = Frame(0,0,0,77);
    auto east = f.EcefPositionToMissionNedM({A,100,0});
    auto north = f.EcefPositionToMissionNedM({A,0,100});
    auto up = f.EcefPositionToMissionNedM({A+50,0,0});
    Check(east.bValid && Near(east.Ned,{0,100,0},1e-12), "east_sign");
    Check(north.bValid && Near(north.Ned,{100,0,0},1e-12), "north_sign");
    Check(up.bValid && Near(up.Ned,{0,0,-50},1e-12), "up_down_sign");
    Check(east.OriginGeneration==77, "origin_generation");
    auto vn=f.EcefVelocityToMissionNedMps({0,0,20}), ve=f.EcefVelocityToMissionNedMps({0,30,0});
    Check(Near(vn.Ned,{20,0,0},1e-12)&&Near(ve.Ned,{0,30,0},1e-12), "velocity_rotation");
    MissionNavigationFrameV2 bad;
    Check(!bad.SetOrigin({0,0,0,1,false}) && !bad.EcefPositionToMissionNedM({1,2,3}).bValid, "invalid_origin");
    Check(!bad.SetOrigin({std::numeric_limits<double>::quiet_NaN(),0,0,1,true}), "nan_origin");
    Check(!f.EcefVelocityToMissionNedMps({INFINITY,0,0}).bValid, "inf_input");

    constexpr std::uint64_t seed=0x4E415632ULL; constexpr int cases=5000;
    std::mt19937_64 rng(seed); std::uniform_real_distribution<double> lat(-1.55,1.55),lon(-Pi,Pi),h(-500,30000),d(-1e5,1e5),v(-1000,1000);
    for(int i=0;i<cases;i++){
        const double la=lat(rng),lo=lon(rng),he=h(rng); bool ok=false;
        const Vec3dV2 e=MissionNavigationFrameV2::GeodeticToEcefM(la,lo,he,ok),o=Oracle(la,lo,he);
        maxEcef=std::max(maxEcef,Norm(e-o)); if(!ok||Norm(e-o)>1e-8){failures++;continue;}
        MissionNavigationFrameV2 frame; if(!frame.SetOrigin({la,lo,he,(std::uint32_t)i,true})){failures++;continue;}
        Vec3dV2 ned{d(rng),d(rng),d(rng)},vel{v(rng),v(rng),v(rng)};
        Vec3dV2 ep=frame.MissionNedPositionToEcefM(ned,ok); auto nr=frame.EcefPositionToMissionNedM(ep);
        maxPositionRoundTrip=std::max(maxPositionRoundTrip,Norm(nr.Ned-ned));
        Vec3dV2 ev=frame.MissionNedVelocityToEcefMps(vel,ok); auto vr=frame.EcefVelocityToMissionNedMps(ev);
        maxVelocityRoundTrip=std::max(maxVelocityRoundTrip,Norm(vr.Ned-vel));
        if(!ok||!nr.bValid||!vr.bValid||nr.OriginGeneration!=(std::uint32_t)i||Norm(nr.Ned-ned)>2e-9||Norm(vr.Ned-vel)>1e-10) failures++;
    }
    std::cout<<std::setprecision(12)<<"MISSION_NAV_V2_AUDIT checks="<<checks<<" seed="<<seed<<" cases="<<cases<<" failures="<<failures
             <<" max_ecef_error_m="<<maxEcef<<" max_position_roundtrip_m="<<maxPositionRoundTrip<<" max_velocity_roundtrip_mps="<<maxVelocityRoundTrip<<'\n';
    return failures?1:0;
}
