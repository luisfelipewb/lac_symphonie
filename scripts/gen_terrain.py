#!/usr/bin/env python3
# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Build the lac_symphonie_terrain model from the data downloaded by fetch_data.py.

Pipeline (numpy / scipy / PIL only):
  1. DTM pixel grid -> local ENU x/y through lac_symphonie.geo (same maths as the navsat)
  2. rasterize the OSM lake polygon (outer ring minus island)
  3. z = DTM - water level (median DTM inside the polygon, ~191.74 m) -> water surface is z = 0
  4. carve a synthetic lake bed (bowl from the distance to the shore) or interpolate a
     bathymetry CSV (--bathymetry lat,lon,depth)
  5. clamp the land to z >= LAND_MIN_Z so the 6 km coast_waves plane does not show through
     the stream valley north of the lake
  6. write meshes/terrain.obj (visual, orthophoto UVs) + meshes/terrain_collision.obj
     (coarser), materials/textures/ortho.jpg, model.sdf / model.config
  7. write models/lac_symphonie_water: a lake-shaped flat mesh at z = 0 carrying the
     VRX Gerstner wave shader (materials copied from vrx_gz/models/coast_waves), so
     the world does not need the 6 km coast_waves sea plane
  8. data/terrain_preview.png: hillshade + shoreline for a visual check;
     data/terrain_grid.npz: the final z grid, read by gen_trees.py

Outputs go to models/lac_symphonie_terrain/ and models/lac_symphonie_water/ and are
meant to be committed.
"""

import argparse
import json
import os
import shutil
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from lac_symphonie import geo  # noqa: E402

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
DATA = os.path.join(PKG, 'data')
MODEL = os.path.join(PKG, 'models', 'lac_symphonie_terrain')
WATER = os.path.join(PKG, 'models', 'lac_symphonie_water')

VISUAL_STEP_M = 2.5       # grid spacing of the visual mesh (~230k tris, ~15 MB OBJ)
COLLISION_STEP_M = 4.0    # grid spacing of the collision mesh
LAND_MIN_Z = 0.3          # with --clamp: land never below this (only needed with the coast_waves sea plane)
WATER_MARGIN_M = 4.0      # lake water mesh extends this far under the bank
MAX_DEPTH = 2.5           # synthetic bowl: depth = min(MAX_DEPTH, SHORE_DEPTH + SLOPE * dist_to_shore)
SHORE_DEPTH = 0.4
SLOPE = 0.25
BANK_SMOOTH_PX = 3        # gaussian sigma applied on the shore transition band only
Image.MAX_IMAGE_PIXELS = None


def load_inputs():
    with open(os.path.join(DATA, 'bbox.json')) as f:
        bbox = json.load(f)
    n = bbox['dem_px']
    dtm = np.fromfile(os.path.join(DATA, 'dtm.bin'), dtype='<f4').reshape(n, n)
    with open(os.path.join(DATA, 'lake.json')) as f:
        rel = json.load(f)['elements'][0]
    return bbox, dtm, rel


def pixel_grid(bbox, n):
    """lat/lon of pixel centres (row 0 = north) and their local x/y."""
    lats = bbox['lat_n'] - (np.arange(n) + 0.5) / n * (bbox['lat_n'] - bbox['lat_s'])
    lons = bbox['lon_w'] + (np.arange(n) + 0.5) / n * (bbox['lon_e'] - bbox['lon_w'])
    conv = geo.make_converter()
    # ENU over < 1 km is separable to well below 1 mm: convert the axes, not every pixel
    ys = np.array([conv.local_from_latlon(lat, geo.ORIGIN_LON)[1] for lat in lats])
    xs = np.array([conv.local_from_latlon(geo.ORIGIN_LAT, lon)[0] for lon in lons])
    return lats, lons, xs, ys


def latlon_to_px(bbox, n, lat, lon):
    return ((lon - bbox['lon_w']) / (bbox['lon_e'] - bbox['lon_w']) * n,
            (bbox['lat_n'] - lat) / (bbox['lat_n'] - bbox['lat_s']) * n)


def lake_mask(bbox, n, rel):
    img = Image.new('L', (n, n), 0)
    dr = ImageDraw.Draw(img)
    rings = sorted(rel['members'], key=lambda m: m['role'] != 'outer')  # outers first
    for m in rings:
        if m['type'] != 'way':
            continue
        poly = [latlon_to_px(bbox, n, g['lat'], g['lon']) for g in m['geometry']]
        dr.polygon(poly, fill=255 if m['role'] == 'outer' else 0)
    return np.array(img) > 0


def carve_bathymetry(z, mask, px_m, csv_path, bbox, n):
    """Lake bed: interpolate a lat,lon,depth CSV if given, else a synthetic bowl."""
    dist = ndimage.distance_transform_edt(mask) * px_m
    bowl = np.minimum(MAX_DEPTH, SHORE_DEPTH + SLOPE * dist)
    if csv_path:
        from scipy.interpolate import griddata
        pts = np.loadtxt(csv_path, delimiter=',', comments='#', ndmin=2)
        px = np.array([latlon_to_px(bbox, n, la, lo) for la, lo, _ in pts])
        rr, cc = np.nonzero(mask)
        depth = griddata(px[:, ::-1], pts[:, 2], (rr, cc), method='linear')
        nearest = griddata(px[:, ::-1], pts[:, 2], (rr, cc), method='nearest')
        depth = np.where(np.isnan(depth), nearest, depth)
        bed = np.zeros_like(z)
        bed[rr, cc] = -np.maximum(depth, 0.2)
        print(f'  bathymetry from {csv_path}: {len(pts)} soundings, depth {depth.min():.1f}..{depth.max():.1f} m')
    else:
        bed = -bowl
        print(f'  synthetic bowl: max depth {MAX_DEPTH} m, shore {SHORE_DEPTH} m, slope {SLOPE}')
    z = np.where(mask, bed, z)
    # smooth only the shore transition band so the bank does not step
    band = ndimage.binary_dilation(mask, iterations=BANK_SMOOTH_PX * 2) & ~ndimage.binary_erosion(mask, iterations=BANK_SMOOTH_PX * 2)
    zs = ndimage.gaussian_filter(z, BANK_SMOOTH_PX)
    z = np.where(band, zs, z)
    return z


def write_obj(path, xs, ys, z, step_px, uv=None, texture_mtl=None):
    """Regular-grid OBJ (two triangles per cell) with per-vertex normals.

    Normals are mandatory: dartsim's mesh collision drops (and then crashes on)
    submeshes whose normal count differs from the vertex count.
    """
    sub_r = np.arange(0, z.shape[0], step_px)
    sub_c = np.arange(0, z.shape[1], step_px)
    if sub_r[-1] != z.shape[0] - 1:
        sub_r = np.append(sub_r, z.shape[0] - 1)
    if sub_c[-1] != z.shape[1] - 1:
        sub_c = np.append(sub_c, z.shape[1] - 1)
    Z = z[np.ix_(sub_r, sub_c)]
    X, Y = np.meshgrid(xs[sub_c], ys[sub_r])
    nr, nc = Z.shape
    # normal of the height field: (-dz/dx, -dz/dy, 1) normalised
    dzdy, dzdx = np.gradient(Z, ys[sub_r], xs[sub_c])
    N = np.dstack([-dzdx, -dzdy, np.ones_like(Z)])
    N /= np.linalg.norm(N, axis=2, keepdims=True)
    with open(path, 'w') as f:
        f.write('# lac_symphonie terrain, generated by scripts/gen_terrain.py\n')
        if texture_mtl:
            f.write(f'mtllib {texture_mtl}\nusemtl {"ortho" if uv else "plain"}\n')
        np.savetxt(f, np.column_stack([X.ravel(), Y.ravel(), Z.ravel()]), fmt='v %.3f %.3f %.3f')
        np.savetxt(f, N.reshape(-1, 3), fmt='vn %.4f %.4f %.4f')
        if uv:
            U, V = np.meshgrid(sub_c / (z.shape[1] - 1), 1.0 - sub_r / (z.shape[0] - 1))
            np.savetxt(f, np.column_stack([U.ravel(), V.ravel()]), fmt='vt %.5f %.5f')
        idx = np.arange(nr * nc).reshape(nr, nc) + 1
        a = idx[:-1, :-1].ravel(); b = idx[:-1, 1:].ravel()
        c = idx[1:, 1:].ravel(); d = idx[1:, :-1].ravel()
        # counter-clockwise seen from above (+z): a (NW) -> d (SW) -> c (SE), a -> c -> b (NE)
        tris = np.vstack([np.column_stack([a, d, c]), np.column_stack([a, c, b])])
        if uv:
            rows = np.repeat(tris, 3, axis=1)                      # v/vt/vn all share the index
            np.savetxt(f, rows, fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')
        else:
            rows = np.repeat(tris, 2, axis=1)
            np.savetxt(f, rows, fmt='f %d//%d %d//%d %d//%d')
    return len(tris)


def write_water(xs, ys, mask, step_px, px_m):
    """Flat lake-shaped mesh at z = 0 + model carrying the VRX wave shader."""
    grow = ndimage.binary_dilation(mask, iterations=int(round(WATER_MARGIN_M / px_m)))
    sub_r = np.arange(0, mask.shape[0], step_px)
    sub_c = np.arange(0, mask.shape[1], step_px)
    keep = grow[np.ix_(sub_r, sub_c)]
    nr, nc = keep.shape
    X, Y = np.meshgrid(xs[sub_c], ys[sub_r])
    idx = np.arange(nr * nc).reshape(nr, nc) + 1
    a = idx[:-1, :-1]; b = idx[:-1, 1:]; c = idx[1:, 1:]; d = idx[1:, :-1]
    cell = keep[:-1, :-1] & keep[:-1, 1:] & keep[1:, 1:] & keep[1:, :-1]
    tris = np.vstack([np.column_stack([a[cell], d[cell], c[cell]]), np.column_stack([a[cell], c[cell], b[cell]])])
    used, tris = np.unique(tris, return_inverse=True)       # drop the unused (land) vertices
    tris = tris.reshape(-1, 3) + 1
    vx, vy = X.ravel()[used - 1], Y.ravel()[used - 1]
    os.makedirs(os.path.join(WATER, 'meshes'), exist_ok=True)
    with open(os.path.join(WATER, 'meshes', 'lake_water.obj'), 'w') as f:
        f.write('# lac_symphonie lake water surface, generated by scripts/gen_terrain.py\n')
        np.savetxt(f, np.column_stack([vx, vy, np.zeros(len(vx))]), fmt='v %.3f %.3f %.3f')
        f.write('vn 0 0 1\n')
        # UVs in metres / 50 so the wave normal map tiles like on the coast_waves mesh
        np.savetxt(f, np.column_stack([vx / 50.0, vy / 50.0]), fmt='vt %.4f %.4f')
        ones = np.ones(len(tris), int)
        rows = np.column_stack([tris[:, 0], tris[:, 0], ones, tris[:, 1], tris[:, 1], ones, tris[:, 2], tris[:, 2], ones])
        np.savetxt(f, rows, fmt='f %d/%d/%d %d/%d/%d %d/%d/%d')
    # shader programs + textures: WaveVisual looks for them relative to the model dir
    src = coast_waves_dir()
    if os.path.isdir(os.path.join(WATER, 'materials')):
        shutil.rmtree(os.path.join(WATER, 'materials'))
    shutil.copytree(os.path.join(src, 'materials'), os.path.join(WATER, 'materials'))
    sdf = open(os.path.join(src, 'model.sdf')).read()
    sdf = sdf.replace('<model name="waves">', '<model name="lac_symphonie_water">')
    sdf = sdf.replace('<uri>meshes/waterlow.dae</uri>', '<uri>meshes/lake_water.obj</uri>')
    sdf = sdf.replace('<?xml version="1.0"?>', '<?xml version="1.0"?>\n<!-- Generated by scripts/gen_terrain.py from '
                      'vrx_gz/models/coast_waves: same shader, lake-shaped mesh. -->')
    with open(os.path.join(WATER, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(WATER, 'model.config'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<model>
  <name>lac_symphonie_water</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>
    Water surface of Lac Symphonie: flat mesh at z = 0 following the OSM shoreline
    with the VRX Gerstner wave shader (materials copied from vrx_gz coast_waves).
    Generated by scripts/gen_terrain.py.
  </description>
</model>
''')
    return len(tris)


def coast_waves_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('vrx_gz'), 'models', 'coast_waves')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    d = os.path.join(PKG, '..', 'vrx', 'vrx_gz', 'models', 'coast_waves')
    if os.path.isdir(d):
        return d
    sys.exit('cannot find vrx_gz/models/coast_waves (source the workspace or keep vrx next to this package)')


def write_model(stats):
    os.makedirs(os.path.join(MODEL, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(MODEL, 'materials', 'textures'), exist_ok=True)
    with open(os.path.join(MODEL, 'meshes', 'terrain.mtl'), 'w') as f:
        f.write('newmtl ortho\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd ../materials/textures/ortho.jpg\n')
    with open(os.path.join(MODEL, 'meshes', 'terrain_collision.mtl'), 'w') as f:
        f.write('newmtl plain\nKd 0.5 0.5 0.5\n')
    with open(os.path.join(MODEL, 'model.config'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<model>
  <name>lac_symphonie_terrain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Luis Batista</name><email>luisfelipewb@gmail.com</email></author>
  <description>
    Terrain around Lac Symphonie (Metz) generated from the IGN RGE ALTI 1 m DTM
    and the IGN orthophoto (Etalab open licence), shoreline from OpenStreetMap
    (ODbL).  Lake bed is synthetic.  Generated by scripts/gen_terrain.py.
  </description>
</model>
''')
    with open(os.path.join(MODEL, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<!-- Generated by scripts/gen_terrain.py: water level {stats['level']:.2f} m,
     land z {stats['zmin_land']:.2f}..{stats['zmax']:.2f}, lake bed to {stats['zmin_lake']:.2f} m,
     visual {stats['tris_visual']} tris, collision {stats['tris_collision']} tris. -->
<sdf version="1.9">
  <model name="lac_symphonie_terrain">
    <static>true</static>
    <link name="link">
      <visual name="terrain">
        <geometry>
          <mesh><uri>meshes/terrain.obj</uri></mesh>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://lac_symphonie_terrain/materials/textures/ortho.jpg</albedo_map>
              <roughness>0.9</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="terrain_collision">
        <geometry>
          <mesh><uri>meshes/terrain_collision.obj</uri></mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
''')


def preview(z, mask, xs, ys, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource
    ls = LightSource(azdeg=315, altdeg=45)
    fig, ax = plt.subplots(figsize=(8, 8), dpi=110)
    ax.imshow(ls.hillshade(z, vert_exag=3), cmap='gray', extent=[xs[0], xs[-1], ys[-1], ys[0]])
    im = ax.imshow(np.ma.masked_where(~mask, z), cmap='Blues_r', alpha=0.8, extent=[xs[0], xs[-1], ys[-1], ys[0]])
    ax.contour(xs, ys, mask.astype(float), levels=[0.5], colors='cyan', linewidths=0.8)
    ax.plot(0, 0, 'r+', ms=12); ax.set_xlabel('x east [m]'); ax.set_ylabel('y north [m]')
    ax.set_title('lac_symphonie terrain (z rel. water level); cyan = OSM shoreline')
    fig.colorbar(im, ax=ax, shrink=0.7, label='lake bed z [m]')
    fig.savefig(path, bbox_inches='tight'); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bathymetry', help='CSV lat,lon,depth(m) soundings to use instead of the synthetic bowl')
    ap.add_argument('--clamp', action='store_true',
                    help=f'raise land to z >= {LAND_MIN_Z} m (needed only if the world uses the coast_waves sea plane)')
    args = ap.parse_args()

    bbox, dtm, rel = load_inputs()
    n = dtm.shape[0]
    lats, lons, xs, ys = pixel_grid(bbox, n)
    px_m = float(np.mean([abs(xs[1] - xs[0]), abs(ys[1] - ys[0])]))
    print(f'grid {n}x{n}, {px_m:.3f} m/px, x {xs[0]:.0f}..{xs[-1]:.0f}, y {ys[-1]:.0f}..{ys[0]:.0f}')

    mask = lake_mask(bbox, n, rel)
    level = float(np.median(dtm[mask]))
    print(f'lake: {mask.sum() * px_m * px_m / 1e4:.2f} ha, water level {level:.2f} m '
          f'(DTM inside {dtm[mask].min():.2f}..{dtm[mask].max():.2f}); world origin elevation {geo.ORIGIN_ALT}')
    if abs(level - geo.ORIGIN_ALT) > 0.2:
        print('  WARNING: water level differs from geo.ORIGIN_ALT / the world <elevation>; update them')
    z = dtm - geo.ORIGIN_ALT
    z = carve_bathymetry(z, mask, px_m, args.bathymetry, bbox, n)
    if args.clamp:
        low = (~mask) & (z < LAND_MIN_Z)
        print(f'  clamping {low.mean() * 100:.1f} % of the land to z >= {LAND_MIN_Z} m')
        z = np.where(low, LAND_MIN_Z, z)

    vis_step = max(1, int(round(VISUAL_STEP_M / px_m)))
    col_step = max(1, int(round(COLLISION_STEP_M / px_m)))
    os.makedirs(os.path.join(MODEL, 'meshes'), exist_ok=True)
    tv = write_obj(os.path.join(MODEL, 'meshes', 'terrain.obj'), xs, ys, z, vis_step, uv=True, texture_mtl='terrain.mtl')
    tc = write_obj(os.path.join(MODEL, 'meshes', 'terrain_collision.obj'), xs, ys, z, col_step, texture_mtl='terrain_collision.mtl')
    tw = write_water(xs, ys, mask, vis_step, px_m)
    os.makedirs(os.path.join(MODEL, 'materials', 'textures'), exist_ok=True)
    shutil.copyfile(os.path.join(DATA, 'ortho.jpg'), os.path.join(MODEL, 'materials', 'textures', 'ortho.jpg'))
    stats = {'level': level, 'zmin_land': float(z[~mask].min()), 'zmax': float(z.max()),
             'zmin_lake': float(z[mask].min()), 'tris_visual': tv, 'tris_collision': tc}
    write_model(stats)
    np.savez_compressed(os.path.join(DATA, 'terrain_grid.npz'), z=z, mask=mask, xs=xs, ys=ys)  # for gen_trees.py
    preview(z, mask, xs, ys, os.path.join(DATA, 'terrain_preview.png'))
    print(f'visual {tv} tris ({vis_step * px_m:.1f} m), collision {tc} tris ({col_step * px_m:.1f} m); '
          f'land z {stats["zmin_land"]:.2f}..{stats["zmax"]:.2f}, lake bed {stats["zmin_lake"]:.2f} m')
    print(f'water mesh {tw} tris; wrote {os.path.relpath(MODEL)}, {os.path.relpath(WATER)} and data/terrain_preview.png')


if __name__ == '__main__':
    main()
