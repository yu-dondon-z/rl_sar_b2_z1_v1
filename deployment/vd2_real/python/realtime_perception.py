"""Rear seam geometry from measured ground, with explicit small depth holes.

No missing depth is replaced in the sensor array. RGB rays over locally
supported holes intersect the measured plane, and are counted separately from
the directly depth-supported seam points. Foreground and large unknown regions
remain excluded. Confidence and direct-depth limits still apply. Rear validity
uses measured support on both sides of the tool work point, rather than the
front path's global 12-cm length requirement.
"""
import cv2
import numpy as np
from dual_camera_perception import (camera_geometry,fit_ground_plane,unproject,
    project_to_plane,rear_ground_roi,intersect_pixels_with_plane)
from vd1_visual_rear import SeamOnlyDualPerception


def rear_topology_roi(depth,allowed,intrinsic,transform,plane,config):
    """Unknown depth may preserve RGB topology, never supply 3-D evidence.

    Measured non-ground and the dilated foreground exclusion still break the
    path, including missing pixels adjacent to a measured foreground object.
    """
    v,u=np.indices(depth.shape,dtype=float)
    nr=plane[:3]@transform[:3,:3]@np.linalg.inv(intrinsic)
    height=depth*(nr[0]*u+nr[1]*v+nr[2])+plane[:3]@transform[:3,3]+plane[3]
    elevated=np.isfinite(depth)&(height>config.rear_max_foreground_height_m)
    margin=config.rear_foreground_margin_px
    if margin:
        elevated=cv2.dilate(elevated.astype(np.uint8),cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,(2*margin+1,2*margin+1)))>0
    return (allowed|~np.isfinite(depth))&~elevated


def supported_depth_holes(depth,allowed,intrinsic,transform,plane,config):
    measured=np.isfinite(depth)
    v,u=np.indices(depth.shape,dtype=float)
    nr=plane[:3]@transform[:3,:3]@np.linalg.inv(intrinsic)
    denominator=nr[0]*u+nr[1]*v+nr[2]
    offset=plane[:3]@transform[:3,3]+plane[3]
    predicted=np.full(depth.shape,np.nan)
    np.divide(-offset,denominator,out=predicted,where=np.abs(denominator)>1e-8)
    height=depth*denominator+offset
    elevated=measured&(height>config.rear_max_foreground_height_m)
    margin=config.rear_foreground_margin_px
    if margin:
        elevated=cv2.dilate(elevated.astype(np.uint8),cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,(2*margin+1,2*margin+1)))>0
    # At most a 25-pixel patch (12 px radius); also require its predicted
    # physical radius <= 3 cm. Unknown borders/large holes cannot be filled.
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(25,25))
    closed=cv2.morphologyEx(allowed.astype(np.uint8),cv2.MORPH_CLOSE,kernel)>0
    density=cv2.boxFilter(allowed.astype(np.float32),-1,(25,25),borderType=cv2.BORDER_CONSTANT)
    supported=(~measured)&closed&(density>=.35)&(~elevated)
    supported&=(predicted>=config.min_depth_m)&(predicted<=config.max_depth_m)
    supported&=predicted*12/min(intrinsic[0,0],intrinsic[1,1])<=.03
    supported[:12]=supported[-12:]=False
    supported[:,:12]=supported[:,-12:]=False
    return allowed|supported,supported


def rear_component(mask,config,anchor,extract,intrinsic,transform,plane,previous_y=None,max_change=.1525):
    """Prefer an actually observed component spanning the actor's work point."""
    from crack_trajectory.centerline import clean_binary_mask
    clean=clean_binary_mask(mask,config)
    count,labels,stats,_=cv2.connectedComponentsWithStats(clean,connectivity=8)
    candidates=[]
    for label in range(1,count):
        try:
            trajectory=extract((labels==label).astype(np.uint8)*255,config,anchor)
            uv=np.asarray(trajectory.waypoints_uv,dtype=float)
            points,valid=intersect_pixels_with_plane(uv,intrinsic,transform,plane)
            points=points[valid]
            if len(points)<6:continue
            covers=bool(points[:,0].min()<=-1.02<=points[:,0].max())
            if covers:
                order=np.argsort(points[:,0],kind='stable')
                y=float(np.interp(-1.02,points[order,0],points[order,1]))
                # Track observed continuity, not actuator reachability. A real
                # seam remains observable when it leaves the actor workspace.
                if previous_y is not None and abs(y-previous_y)>max_change:continue
            distance=float(np.min(np.sum((uv-np.asarray(anchor))**2,axis=1)))
            # A shorter fragment closer to the image anchor must not hide a
            # separately measured component that contains the actual work x.
            candidates.append(((covers,-distance,float(np.ptp(points[:,0]))),trajectory))
        except (ValueError,RuntimeError,cv2.error):continue
    if not candidates:raise ValueError('no usable observed rear seam component')
    return max(candidates,key=lambda c:c[0])[1]


def rear_work_section(result,points,uv,direct,confidence,config):
    """Validate local interpolation from real depth, without inventing a path.

    A short measured section can locate the work point. Require six points,
    two direct depth samples on each side, 2 cm support on either side and a
    <=2.5 cm measured bracket. Missing work-point coverage still fails.
    """
    work_x=-1.02
    if len(points)<config.min_path_points:
        raise ValueError('rear seam target section is too short')
    if points[-1,0]<points[0,0]:
        points,uv,direct=points[::-1],uv[::-1],direct[::-1]
    steps=np.linalg.norm(np.diff(points,axis=0),axis=1)
    observed=points[direct]
    result.update(rear_validation_mode='observed_work_point_bracket',
        rear_validation_point_count=int(len(points)),
        rear_path_x_range_m=[float(points[:,0].min()),float(points[:,0].max())],
        rear_direct_depth_x_range_m=([float(observed[:,0].min()),float(observed[:,0].max())]
                                    if len(observed) else None),
        path_length_m=float(steps.sum()),rear_work_x_m=work_x,
        rear_required_support_each_side_m=.02,rear_max_work_bracket_m=.025)
    if np.max(steps)>.25:
        raise ValueError('seam geometry contains a gap larger than 0.25 m')
    left=observed[observed[:,0]<work_x]
    right=observed[observed[:,0]>work_x]
    if (len(left)<2 or len(right)<2 or left[:,0].min()>work_x-.02
            or right[:,0].max()<work_x+.02):
        raise ValueError('rear work x=-1.02 m lacks direct depth support on both sides')
    before=left[np.argmax(left[:,0])];after=right[np.argmin(right[:,0])]
    gap=float(np.linalg.norm(after-before))
    result['rear_observed_work_bracket_m']=gap
    if gap>.025:
        raise ValueError('rear work point measured depth bracket exceeds 2.5 cm')
    result.update(points_b=points.tolist(),control_waypoints_robot_xyz=points.tolist(),
        waypoints_uv=uv.tolist(),confidence=float(confidence),mean_waypoint_confidence=float(confidence))


class ObservableRearPerception(SeamOnlyDualPerception):
    def process_rear(self,metadata,rgb_bgr,depth):
        result=self._base_result(metadata)
        result.update(tool_target_mode='rear_seam_direct',visual_tcp_feedback=False,requires_tcp_markers=False)
        original=self._extract_centerline
        self.last_filtered_mask=None
        if self.segmenter is not None:self.segmenter.last_diagnostic_mask=None
        prior=getattr(self,'_accepted_rear',None)
        capture=metadata['capture_timestamp'];generation=metadata['capture_generation']
        elapsed=capture-prior[1] if prior is not None else None
        previous_y=(prior[2] if prior is not None and prior[0]==generation and 0<elapsed<=.6 else None)
        max_change=.04+.45*min(max(elapsed or 0.,0.),.25)
        try:
            intrinsic,transform=camera_geometry(metadata)
            depth_m=self._depth_m(metadata,rgb_bgr,depth)
            v,u=np.mgrid[:depth_m.shape[0]:6,:depth_m.shape[1]:6]
            sampled=depth_m[v,u];valid=np.isfinite(sampled)
            points=unproject(np.c_[u[valid],v[valid]],sampled[valid],intrinsic,transform)
            plane,diagnostics=fit_ground_plane(points,self.config)
            result.update(diagnostics,ground_plane_b=plane.tolist())
            allowed,diagnostics=rear_ground_roi(depth_m,intrinsic,transform,plane,self.config)
            result.update(diagnostics)
            topology=rear_topology_roi(depth_m,allowed,intrinsic,transform,plane,self.config)
            allowed,holes=supported_depth_holes(depth_m,allowed,intrinsic,transform,plane,self.config)
            result.update(rear_plane_supported_hole_pixels=int(holes.sum()),
                rear_component_selection='observed_work_x_before_image_anchor')
            def extract_observed(mask,config,anchor):
                self.last_filtered_mask=np.where(allowed,mask,0).astype(np.uint8)
                return rear_component(mask,config,anchor,original,intrinsic,transform,plane,previous_y,max_change)
            self._extract_centerline=extract_observed
            uv,confidence,diagnostics=self._centerline(rgb_bgr,.65,topology)
            result['rear_topology_source']='rgb_with_foreground_exclusion; unknown_depth_not_measurement'
            seam_depth=np.where(allowed,depth_m,np.nan)
            depths=np.full(len(uv),np.nan)
            for i,(x,y) in enumerate(np.rint(uv).astype(int)):
                if 1<=x<depth_m.shape[1]-1 and 1<=y<depth_m.shape[0]-1:
                    if not allowed[y,x]:continue
                    patch=seam_depth[y-1:y+2,x-1:x+2];good=patch[np.isfinite(patch)]
                    if len(good)>=3:depths[i]=np.median(good)
            direct=np.isfinite(depths);ratio=float(np.mean(direct))
            if ratio<self.config.min_valid_depth_ratio:
                raise ValueError('rear seam direct depth support too low (%.3f)'%ratio)
            points,ray_valid=intersect_pixels_with_plane(uv,intrinsic,transform,plane)
            pixels=np.rint(uv).astype(int)
            # A ray without its own valid depth is accepted only inside the
            # explicitly bounded, depth-surrounded hole region above.
            fallback=(~direct)&holes[pixels[:,1],pixels[:,0]]
            use=ray_valid&(direct|fallback)
            # Preserve measured depth projection for directly observed points.
            points[direct]=project_to_plane(unproject(uv[direct],depths[direct],intrinsic,transform),plane)
            local=use&(points[:,0]>=self.config.rear_validation_x_min_m)&(points[:,0]<=self.config.rear_validation_x_max_m)
            if np.count_nonzero(local)<self.config.min_path_points:
                raise ValueError('rear seam target section is too short')
            rear_work_section(result,points[local],uv[local],direct[local],confidence,self.config)
            ordered=points[local][np.argsort(points[local,0],kind='stable')]
            y=float(np.interp(-1.02,ordered[:,0],ordered[:,1]))
            result.update(rear_observed_work_y_m=y,
                          rear_within_actor_lateral_workspace=bool(-.15<=y<=.15))
            if previous_y is not None and abs(y-previous_y)>max_change:
                raise ValueError('rear measured seam differs from previous accepted component')
            self._accepted_rear=(generation,capture,y)
            result.update(diagnostics,valid=True,reason='ok',valid_3d_ratio=ratio,
                direct_depth_point_count=int(np.count_nonzero(local&direct)),
                plane_intersection_point_count=int(np.count_nonzero(local&fallback)),
                geometry_source='rear_rgb_depth_with_bounded_measured_plane_holes',
                rear_validation_x_range_m=[self.config.rear_validation_x_min_m,self.config.rear_validation_x_max_m],
                rear_validation_point_count=int(np.count_nonzero(local)))
        except (KeyError,ValueError,RuntimeError,cv2.error) as error:result['reason']=str(error)
        finally:self._extract_centerline=original
        return result
