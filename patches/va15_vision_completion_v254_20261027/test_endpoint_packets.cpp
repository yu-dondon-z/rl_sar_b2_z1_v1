#include "production/near_visual_io.hpp"
#include <cassert>
#include <iostream>
int main() {
    using namespace va15_visual;
    const std::string nonce(32,'a');
    YAML::Node n;
    n["schema"]="va15_long_visual_v1";n["session_id"]=nonce;n["calibration_sha256"]=CalibrationSHA;
    n["timestamp_basis"]="host_receive_monotonic";n["image_width"]=1920;n["image_height"]=1080;
    n["sequence"]=1;n["capture_monotonic_ns"]=9900000000ULL;n["publish_monotonic_ns"]=9999000000ULL;
    n["valid"]=true;n["image_received"]=true;n["near_u"]=935.;n["near_v"]=972.;
    n["confidence"]=.99;n["native_width_px"]=30.;n["contrast"]=.8;n["native_fit_rms_px"]=1.;
    n["look_u"]=935.;n["look_v"]=820.8;n["look_confidence"]=.99;
    n["path_point_count"]=51;n["path_vertical_span_px"]=300.;
    for(int v=1044;v>=744;v-=6) {YAML::Node uv;uv.push_back(935.);uv.push_back(double(v));n["path_points_uv"].push_back(uv);}
    n["endpoint_candidate"]=true;n["endpoint_u"]=935.;n["endpoint_v"]=744.;
    n["endpoint_confidence"]=.99;n["endpoint_clear_pixels"]=126.;
    const auto parse=[&](const YAML::Node& node){return Parse(YAML::Dump(node),nonce);};
    const auto p=parse(n);assert(Valid(p,10000000000ULL)&&p.endpoint_candidate&&p.image_received);
    for(int bad=0;bad<6;++bad) {
        auto q=YAML::Clone(n);
        if(bad==0)q["endpoint_u"]=1000.;
        if(bad==1)q["endpoint_v"]=730.;
        if(bad==2)q["endpoint_confidence"]=NAN;
        if(bad==3)q["endpoint_clear_pixels"]=0.;
        if(bad==4)q["image_received"]=false;
        if(bad==5)q["endpoint_confidence"]=1.1;
        assert(!parse(q).valid);
    }
    // An explicitly invalid image can still prove camera liveness, never a cap.
    auto invalid=YAML::Clone(n);invalid["valid"]=false;
    const auto blank=parse(invalid);assert(blank.image_received&&!blank.valid&&!blank.endpoint_candidate);
    for(bool tolerant:{false,true}) {
        Guide g(true,true,tolerant);
        g.Observe(p,10000000000ULL);
        auto changed=p;changed.endpoint_confidence=.98;g.Observe(changed,10000000000ULL);
        assert(g.qualified==0&&!g.latest.valid);
    }
    std::cout<<"PASS endpoint packet geometry, finite values, image liveness and modified-duplicate rejection\n";
}
