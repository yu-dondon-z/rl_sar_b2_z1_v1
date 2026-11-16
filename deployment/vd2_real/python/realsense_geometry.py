"""RealSense lens geometry, using the installed SDK rather than OpenCV's model.

The map is destination (rectified pinhole) -> source (sensor colour image).
It is built once per stream and shared by RGB and aligned depth. In particular,
inverse/modified Brown-Conrady coefficients are not OpenCV Brown coefficients.
"""
import numpy as np


def rectification_maps(intrinsics, sdk=None):
    if sdk is None:
        import pyrealsense2 as sdk
    supported=(sdk.distortion.none,sdk.distortion.brown_conrady,
               sdk.distortion.modified_brown_conrady,sdk.distortion.inverse_brown_conrady)
    if intrinsics.model not in supported:
        raise ValueError('unsupported D435i colour distortion model: '+str(intrinsics.model))
    values=np.asarray([intrinsics.fx,intrinsics.fy,intrinsics.ppx,intrinsics.ppy,
                       *intrinsics.coeffs],dtype=float)
    if (intrinsics.width<=0 or intrinsics.height<=0 or not np.isfinite(values).all()
            or intrinsics.fx<=0 or intrinsics.fy<=0):
        raise ValueError('invalid RealSense colour intrinsics')
    v,u=np.indices((intrinsics.height,intrinsics.width),dtype=np.float32)
    if intrinsics.model==sdk.distortion.none or not np.any(intrinsics.coeffs):
        return u,v
    rays=np.stack([(u-intrinsics.ppx)/intrinsics.fx,
                   (v-intrinsics.ppy)/intrinsics.fy,np.ones_like(u)],axis=-1)
    # Let the installed SDK interpret its own distortion model. The lookup is
    # generated at startup, not once per frame (640x480 takes ~0.25 s locally).
    pixels=np.asarray([sdk.rs2_project_point_to_pixel(intrinsics,point.tolist())
                       for point in rays.reshape(-1,3)],dtype=np.float32)
    pixels=pixels.reshape(intrinsics.height,intrinsics.width,2)
    if not np.isfinite(pixels).all():
        raise ValueError('RealSense distortion produced non-finite rectification maps')
    return np.ascontiguousarray(pixels[:,:,0]),np.ascontiguousarray(pixels[:,:,1])


def intrinsics_from_record(record, sdk=None):
    """Reconstruct only measured intrinsics saved alongside a raw colour image."""
    if sdk is None:
        import pyrealsense2 as sdk
    models={str(model):model for model in (sdk.distortion.none,sdk.distortion.brown_conrady,
            sdk.distortion.modified_brown_conrady,sdk.distortion.inverse_brown_conrady)}
    model=record['distortion_model']
    if model not in models:
        raise ValueError('unsupported recorded RealSense distortion model: '+str(model))
    k=np.asarray(record['intrinsic_matrix'],dtype=float)
    if k.shape!=(3,3) or not np.isfinite(k).all() or not np.allclose(k[2],[0,0,1]):
        raise ValueError('invalid recorded intrinsic matrix')
    if k[0,1]!=0 or k[1,0]!=0:
        raise ValueError('RealSense intrinsic matrix must have zero skew')
    coeffs=np.asarray(record['distortion_coefficients'],dtype=float)
    if coeffs.shape!=(5,) or not np.isfinite(coeffs).all():
        raise ValueError('expected five finite RealSense distortion coefficients')
    result=sdk.intrinsics()
    result.width,result.height=map(int,record['image_size'])
    result.fx,result.fy=float(k[0,0]),float(k[1,1])
    result.ppx,result.ppy=float(k[0,2]),float(k[1,2])
    result.model=models[model];result.coeffs=coeffs.tolist()
    return result
