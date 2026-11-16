"""Continuous height-field road contact, retaining the rendered crack mesh.

Independent convex road strips expose internal side faces to deeply penetrating
feet. This alternative keeps the physical trench with shared height-field grids
around the robot corridor. It is an explicit simulator change,
not an actor, vision, or robot-parameter adjustment.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

from vd1_mujoco_scene import ROAD_HALF_WIDTH_M, _road_geometry


ROAD_CONTACT = dict(model='continuous_heightfield', x_range_m=[-25., 35.],
                    y_range_m=[-1.75, 1.75], grid_dx_m=.0025, grid_dy_m=.0025,
                    trench_top_width_m=.02, trench_depth_m=.05,
                    trench_side_ramp_m=.0025, heightfield_tile_width_m=.01,
                    rendered_mesh_unchanged=True, legacy_prism_contacts=False)


def continuous_road_contact(scene, curvature):
    """Write a separate MJCF and binary hfield; leave the source scene intact."""
    scene = Path(scene)
    output = scene.with_name('vd1_continuous_road_contact.xml')
    grid_dir = scene.parent/'vd1_road_fields'
    if output.exists() or grid_dir.exists():
        raise FileExistsError('Continuous road assets already exist')
    tree = ET.parse(str(scene))
    root = tree.getroot()
    world, asset = root.find('worldbody'), root.find('asset')
    roads = [g for g in world.findall('geom') if g.get('name', '').startswith('road_')]
    if not roads or world.find("geom[@name='crack_floor']") is None:
        raise ValueError('Expected original physical crack scene')
    # The original rendering remains pixel-identical; only its contact source changes.
    for geom in roads:
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
    # At .40/m, 1cm X samples smear steep lips into the nominal flat surface.
    # Use 2.5mm spacing so the physical edge follows the sharper rendered curve.
    x_lower, x_upper = ROAD_CONTACT['x_range_m']
    x = np.linspace(x_lower, x_upper,
                    round((x_upper-x_lower)/ROAD_CONTACT['grid_dx_m'])+1)
    stations = _road_geometry().road_stations(curvature)
    center = np.interp(x, stations[:, 0], stations[:, 1])
    common = dict(contype='1', conaffinity='2', friction='1 .005 .0001',
                  condim='3', solref='.005 1', rgba='0 0 0 0', group='3', mass='0')
    grid_dir.mkdir()
    # MuJoCo limits each geom/hfield pair to 50 contacts. Narrow tiles keep a
    # deeply penetrating 32mm sphere from exhausting that budget before its
    # vertical support triangles are reached. Adjacent tiles share samples.
    # The .40/m curve reaches +/-1.351m. Keep the full trench inside the
    # height-field corridor so the outer flat boxes cannot fill its bends.
    y_lower, y_upper = ROAD_CONTACT['y_range_m']
    for index in range(round((y_upper-y_lower)/.01)):
        lower = y_lower+index*.01
        y = np.linspace(lower, lower+.01, 5)
        distance = np.abs(y[:, None]-center[None, :])
        # z=0 at each nominal +/-1cm lip, with a 2.5mm side-wall ramp.
        height = np.clip((distance-(.01-.0025))/.0025, 0., 1.).astype('<f4')
        if np.all(height == 1.):
            ET.SubElement(world, 'geom', name='continuous_road_flat_%03d' % index, type='box',
                          pos='5 %.9f -.025' % (lower+.005), size='30 .005 .025', **common)
            continue
        name = 'continuous_road_field_%03d' % index
        grid_file = grid_dir/(name+'.bin')
        with grid_file.open('xb') as handle:
            handle.write(struct.pack('<ii', len(y), len(x)))
            handle.write(height.tobytes(order='C'))
        # Compilation normalizes each file independently. Preserve absolute
        # heights even in outer tiles whose shallow lip never reaches z=-.05.
        low, high = float(height.min()), float(height.max())
        elevation = .05*(high-low)
        ET.SubElement(asset, 'hfield', name=name, file=str(grid_file),
                      size='30 .005 %.12f .05' % elevation)
        ET.SubElement(world, 'geom', name=name, type='hfield', hfield=name,
                      pos='5 %.9f %.12f' % (lower+.005, -.05+.05*low), **common)
    outer_half_width = (ROAD_HALF_WIDTH_M-y_upper)/2
    outer_center = (ROAD_HALF_WIDTH_M+y_upper)/2
    for side in (-1, 1):
        ET.SubElement(world, 'geom', name='continuous_road_outer_%d' % side, type='box',
                      pos='5 %s -.025' % (side*outer_center),
                      size='30 %s .025' % outer_half_width, **common)
    for xmid in (-60., 65.):
        # The curve is identically straight beyond [2,22], including these ends.
        for side in (-1, 1):
            ET.SubElement(world, 'geom', name='continuous_road_end_%s_%s' % (xmid, side),
                          type='box', pos='%s %s -.025' % (xmid, side*1.505),
                          size='%s 1.495 .025' % (35. if xmid < 0 else 30.), **common)
    tree.write(str(output), encoding='utf-8', xml_declaration=True)
    return output
