#pragma once
#include <array>
#include <cmath>
#include <cstdint>

// Pure geometry/validity checks; no sensors, sockets or actuator interfaces.
namespace vision_ground_projection {
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>; // base -> odom, wxyz
inline bool Finite(const Vec3& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Vec3 t{{2*(q[2]*v[2]-q[3]*v[1]), 2*(q[3]*v[0]-q[1]*v[2]),
                  2*(q[1]*v[1]-q[2]*v[0])}};
    return {{v[0]+q[0]*t[0]+q[2]*t[2]-q[3]*t[1],
             v[1]+q[0]*t[1]+q[3]*t[0]-q[1]*t[2],
             v[2]+q[0]*t[2]+q[1]*t[1]-q[2]*t[0]}};
}
inline bool Synchronized(std::uint64_t pose_ns, std::uint64_t frame_ns,
                         std::uint64_t now_ns, bool valid) {
    if (!valid || !pose_ns || !frame_ns || frame_ns>now_ns || pose_ns>now_ns)
        return false;
    const auto delta=pose_ns>frame_ns ? pose_ns-frame_ns : frame_ns-pose_ns;
    return delta<=30000000ULL && now_ns-frame_ns<=500000000ULL;
}
inline bool Project(const Vec3& base_position, const Quat& rotation,
                    const Vec3& camera_origin_base, const Vec3& direction_base,
                    double ground_z, Vec3* point_base) {
    if (!point_base || !Finite(base_position) || !Finite(camera_origin_base) ||
        !Finite(direction_base) || !std::isfinite(ground_z)) return false;
    double norm=0.;
    for (double v:rotation) {if(!std::isfinite(v)) return false; norm+=v*v;}
    if (std::abs(norm-1.)>1.e-4) return false;
    const double height=base_position[2]-ground_z;
    if (height<.40 || height>.80) return false;
    const auto up=Rotate(rotation, {{0.,0.,1.}});
    if (up[2]<std::cos(20.*3.141592653589793/180.)) return false;
    const auto origin=Rotate(rotation,camera_origin_base);
    const auto ray=Rotate(rotation,direction_base);
    const double ray_norm=std::sqrt(ray[0]*ray[0]+ray[1]*ray[1]+ray[2]*ray[2]);
    if (ray_norm<1.e-9 || ray[2]/ray_norm>=-.05) return false;
    const double scale=(ground_z-base_position[2]-origin[2])/ray[2];
    if (!std::isfinite(scale) || scale<=0. || scale*ray_norm>5.) return false;
    for(int i=0;i<3;++i) (*point_base)[i]=camera_origin_base[i]+scale*direction_base[i];
    return Finite(*point_base);
}
} // namespace vision_ground_projection
