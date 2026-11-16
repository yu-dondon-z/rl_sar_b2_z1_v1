#include "production/near_visual_io.hpp"
#include "production/visual_review.hpp"
#include <cassert>
#include <iostream>

using namespace va15_visual;
Packet Frame(unsigned n,std::uint64_t now,double u=935.) {
    Packet p;p.valid=p.session_matched=true;p.sequence=n;
    p.capture_ns=now-100000000;p.publish_ns=now-1000000;
    p.u=u;p.v=972.;p.confidence=.99;p.width=30.;p.contrast=.8;p.rms=1.;
    p.look_u=u;p.look_v=.76*1080.;p.look_confidence=.99;p.path_count=50;p.path_span=300.;return p;
}
void Fill(Guide& g,std::uint64_t now,double u=935.) {
    for(unsigned n=1;n<=6;++n)g.Observe(Frame(n,now-(6-n)*150000000,u),now-(6-n)*150000000);
}
int main() {
    constexpr std::uint64_t now=10000000000ULL;
    assert(Bearing(900.,972.)>0. && Bearing(980.,972.)<0.);
    assert(!std::isfinite(Bearing(935.,200.)));
    auto p=Frame(1,now);assert(Valid(p,now));
    auto q=p;q.session_matched=false;assert(!Valid(q,now));
    q=p;q.publish_ns=now+1;assert(!Valid(q,now));
    q=p;q.width=3;assert(!Valid(q,now));
    q=p;q.width=144;assert(Valid(q,now));
    q=p;q.width=181;assert(!Valid(q,now));
    q=p;q.path_count=0;assert(!Valid(q,now));
    q=p;q.look_confidence=.89;assert(!Valid(q,now));
    assert(!Valid(p,now+MaximumAgeNs));
    Guide g;Fill(g,now);assert(g.Ready(now));
    Guide offset;Fill(offset,now,860.);assert(offset.Ready(now));
    assert(offset.Start(now,true,true,true));
    auto turn=offset.Tick(now+5000000,.005,true,false,true);
    assert(turn.yaw>0.&&turn.yaw<=.000201);
    Guide remote;Fill(remote,now,700.);assert(!remote.Ready(now));
    for(int n=0;n<100;++n)g.Observe(Frame(6,now),now);
    assert(g.qualified==6);
    q=Frame(6,now,940.);g.Observe(q,now);assert(!g.Ready(now));
    Guide good;Fill(good,now);assert(!good.Start(now,false,true,true));
    assert(!good.Start(now,true,false,true));assert(!good.Start(now,true,true,false));
    assert(good.Start(now,true,true,true));
    auto out=good.Tick(now+5000000,.005,true,false,true);
    assert(out.x==Speed && std::fabs(out.yaw)<=YawSlew*.005);
    assert(good.Tick(now+RequestNs,.005,true,false,true).stop);
    assert(!good.Start(now+RequestNs,true,true,true));
    for(int fault=0;fault<5;++fault) {
        Guide item;Fill(item,now);assert(item.Start(now,true,true,true));
        auto stop=item.Tick(now+(fault==0?MaximumAgeNs:5000000),fault==1?.03:.005,
                            fault!=2,fault==3,fault!=4);
        assert(stop.stop && stop.x==0. && stop.yaw==0.);
    }
    Guide jump;Fill(jump,now);assert(jump.Start(now,true,true,true));
    jump.Observe(Frame(7,now+150000000,1100.),now+150000000);
    assert(jump.Tick(now+150000000,.005,true,false,true).stop);
    const std::string nonce(32,'a');
    std::string nodes="[";for(int k=0;k<26;++k)nodes+=(k?",":"")+std::string("[935,")+std::to_string(1050-k*14)+"]";nodes+="]";
    const auto raw=std::string("{schema: va15_long_visual_v1, session_id: ")+nonce+
        ", calibration_sha256: "+CalibrationSHA+", timestamp_basis: host_receive_monotonic, image_width: 1920, image_height: 1080, sequence: 1, capture_monotonic_ns: 9900000000, publish_monotonic_ns: 9999000000, valid: true, near_u: 935, near_v: 972, confidence: 0.99, native_width_px: 30, contrast: 0.8, native_fit_rms_px: 1, look_u: 935, look_v: 820.8, look_confidence: 0.99, path_point_count: 26, path_vertical_span_px: 350, path_points_uv: "+nodes+"}";
    assert(Valid(Parse(raw,nonce),now));assert(!Parse(raw,std::string(32,'b')).session_matched);
    assert(!Parse("[unclosed",nonce).valid);
    assert(!Parse(std::string(17000,'a'),nonce).valid);
    auto mismatched=raw;mismatched.replace(mismatched.find("look_u: 935"),11,"look_u: 700");
    assert(!Parse(mismatched,nonce).valid);
    Guide source;Fill(source,now,860.);Review review;
    assert(review.Start(source,now,true,true,true));
    for(unsigned k=1;k<=800;++k) {
        const auto t=now+k*5000000ULL;
        if(k%30==0) {auto f=Frame(6+k/30,t,860.);source.Observe(f,t);review.Observe(f,t);}
        review.audit.Notify(t,2000000,t-1000000,true);
        review.Tick(source,t,.005,true,true,false,true);
        assert(source.stage==Guide::Stage::Idle); // never automatic motion
    }
    assert(review.approved&&review.audit.Samples()>=100);
    assert(source.Start(now+RequestNs+5000000ULL,true,true,true)); // explicit SECOND request only
    // Approval never survives a stale path or an operator stop.
    review.Tick(source,now+RequestNs+MaximumAgeNs+1,.005,true,true,false,true);
    assert(!review.approved);
    for(int bad=0;bad<4;++bad) {
        Guide s;Fill(s,now);Review r;assert(r.Start(s,now,true,true,true));
        r.audit.Notify(now+5000000,bad==0?30000000:2000000,now-1000000,bad!=1);
        r.Tick(s,now+5000000,.005,bad!=2,true,bad==3,true);
        assert(!r.approved&&r.guide.stage==Guide::Stage::Stopped&&s.stage==Guide::Stage::Idle);
        // A failed zero-output review may be explicitly retried; the real
        // one-shot guide has not been consumed by review-only activity.
        assert(r.Start(s,now+6000000,true,true,true));
    }
    std::cout<<"PASS: near-bearing signs, freshness, replay, entry, stop, time budget, slew, parser/session\n";
}
