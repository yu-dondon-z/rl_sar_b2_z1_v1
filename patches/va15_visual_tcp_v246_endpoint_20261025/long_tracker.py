"""Extend a validated near tape along supported RGB/neural curve evidence.

Pixel path only. No fabricated depth, world odometry, or autonomous motion.
"""
import numpy as np
from near_tracker import NearTracker
from endpoint_vision import visible_endpoint


class LongTracker(NearTracker):
    LOOK_V = .76*1080
    MAX_LOOK_V = .82*1080

    def detect(self,image,mask,probability,timestamp_ns=None):
        near=super().detect(image,mask,probability,timestamp_ns)
        gray,prob,dark=self.frame_fields
        points=list(near['points'])
        seed_width=near['native_width_px']
        previous=np.asarray(points[-1],dtype=float)/3.
        slope=(points[-1][0]-points[-4][0])/(points[-1][1]-points[-4][1])
        missing=0;far_conf=[]
        # Camera-fixed nominal horizon only scales expected IMAGE width; it
        # does not supply ground depth/metric coordinates or pose authority.
        horizon=492.3
        for v in range(int(previous[1])-2,int(.57*360)-1,-2):
            row=dark[v].copy();row[:128]=False;row[512:]=False
            z=np.diff(np.r_[False,row,False].astype(np.int8))
            starts,ends=np.flatnonzero(z==1),np.flatnonzero(z==-1)
            prediction=previous[0]+slope*(v-previous[1])
            expected=seed_width*max(1.,v*3.-horizon)/(near['near_v']-horizon)
            choices=[]
            for a,b in zip(starts,ends):
                width=(b-a)*3.;u=.5*(a+b-1)
                if not max(6.,.40*expected)<=width<=min(180.,2.2*expected):continue
                if abs(u-prediction)>min(10.,4.+2.*missing):continue
                flank=np.r_[gray[v,max(0,a-8):max(0,a-2)],gray[v,min(640,b+2):min(640,b+8)]]
                if len(flank)<4:continue
                background=float(np.median(flank))
                contrast=(background-float(np.median(gray[v,a:b])))/max(1.,background)
                confidence=float(np.median(prob[v,a:b]));coverage=float(np.mean(prob[v,a:b]>=.90))
                if contrast<.25 or confidence<.90 or coverage<.50:continue
                score=-abs(u-prediction)-.03*abs(width-expected)+confidence
                choices.append((score,u,confidence))
            if not choices:
                missing+=1
                if missing>3:break
                continue
            _,u,confidence=max(choices)
            next_slope=(u-previous[0])/(v-previous[1])
            if abs(next_slope-slope)>1.5:break
            slope=.6*slope+.4*next_slope
            previous=np.array([u,v]);missing=0
            points.append([u*3.,v*3.]);far_conf.append((v*3.,confidence))
        path=np.asarray(points,dtype=float)
        span=float(path[0,1]-path[-1,1])
        # Fixed-row lookahead made a still visible 200+ px tape disappear
        # whenever its supported far end stopped a few pixels below 820.8.
        # Choose a point INSIDE this frame's observed continuous curve, with
        # a 12 px supported margin. Never extrapolate to a missing far row.
        look_v=max(self.LOOK_V,float(path[-1,1])+12.)
        look_supported=bool(look_v<=self.MAX_LOOK_V and path[-1,1]<=look_v<=path[0,1])
        look_u=float(np.interp(look_v,path[::-1,1],path[::-1,0])) if look_supported else near['near_u']
        close_conf=[c for v,c in far_conf if abs(v-look_v)<=18.]
        look_conf=float(np.mean(close_conf)) if close_conf else 0.
        # Equal arc spacing retains bends rather than replacing the curve by
        # one fitted straight line. Cap packet size and forbid extrapolation.
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        anchors=[float(np.interp(row,path[::-1,1],arc[::-1]))
                 for row in (near['near_v'],look_v) if path[-1,1]<=row<=path[0,1]]
        distances=np.unique(np.r_[np.arange(0.,arc[-1],8.),arc[-1],anchors])
        # Preserve both endpoints if a very long polyline needs downsampling.
        if len(distances)>128:
            distances=np.unique(np.r_[np.linspace(0.,arc[-1],128-len(anchors)),anchors])
        resampled=np.c_[np.interp(distances,arc,path[:,0]),np.interp(distances,arc,path[:,1])]
        span=float(resampled[0,1]-resampled[-1,1])
        near.update(points=resampled.tolist(),path_points_uv=np.round(resampled,3).tolist(),
                    path_point_count=len(resampled),path_vertical_span_px=span,
                    look_u=look_u,look_v=look_v,look_confidence=look_conf,
                    long_valid=bool(look_supported and look_conf>=.90 and span>=200. and len(resampled)>=25))
        # Only add endpoint evidence; do not grant path validity or extend the
        # observed path when the physical tape disappears from the image.
        near.update(visible_endpoint(self.frame_fields,path,near['long_valid']))
        if not near['long_valid']:
            near['path_reason']='Need >=200 px supported curve and an observed bounded lookahead point'
        self.debug.update(path_point_count=len(resampled),path_span_px=span,look_supported=bool(look_supported),
                          look_confidence=look_conf,look_v=look_v,adaptive_lookahead=look_v>self.LOOK_V)
        return near
