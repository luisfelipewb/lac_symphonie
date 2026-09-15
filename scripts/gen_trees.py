#!/usr/bin/env python3
# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Place trees around Lac Symphonie from OpenStreetMap and splice them into the world.

Sources (data/trees.json from fetch_data.py):
  natural=tree nodes           one tree each
  natural=tree_row ways        one tree every ROW_SPACING_M along the way
  natural=wood / landuse=forest polygons
                               jittered grid, FILL_SPACING_M, only where the IGN
                               canopy height (MNS - DTM) is >= MIN_CANOPY_M
  shore band (SHORE_BAND_M)    same canopy-driven fill, no OSM polygon needed: OSM
                               maps almost none of the lakeside trees, the IGN
                               surface model has them all; OSM building footprints
                               are masked out
Tree height = canopy height at the spot (clamped) -> size class s/m/l; species
from OSM tags (needleleaved -> pine, else oak).  z = terrain height from
data/terrain_grid.npz (run gen_terrain.py first).

Models: models/oak_tree and models/pine_tree are copied from the gz Fuel cache
(downloaded with `gz fuel download` if missing) with URIs rewritten to
model://, and one scaled, collision-free wrapper model per species/class is
written (models/tree_oak_m, ...).

The <include> lines replace everything between
  <!-- BEGIN generated trees --> and <!-- END generated trees -->
in worlds/lac_symphonie.sdf.  Lines inside the block that contain
"<!-- manual -->" survive re-runs; anything outside the block is untouched.
"""

import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from lac_symphonie import geo  # noqa: E402

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
DATA = os.path.join(PKG, 'data')
MODELS = os.path.join(PKG, 'models')
WORLD = os.path.join(PKG, 'worlds', 'lac_symphonie.sdf')
BEGIN, END = '<!-- BEGIN generated trees -->', '<!-- END generated trees -->'

ROW_SPACING_M = 8.0
FILL_SPACING_M = 9.0
MIN_CANOPY_M = 3.0        # polygon / band fill only where the IGN surface model shows canopy
SHORE_BAND_M = 60.0       # around the shoreline, trees come straight from the IGN canopy model
BUILDING_MARGIN_M = 4.0   # keep canopy-derived trees this far from OSM building footprints
MAX_RADIUS_M = 450.0      # ignore trees farther than this from the origin
TREE_CAP = 600            # nearest trees win
SEED = 7

# Fuel models: (fuel name, local dir, natural height of the mesh in m)
SPECIES = {'oak': ('Oak tree', 'oak_tree', 6.5), 'pine': ('Pine Tree', 'pine_tree', 5.1)}
# size class -> target height in m (mesh is scaled to it)
CLASSES = {'s': 5.0, 'm': 9.0, 'l': 14.0}
FUEL_CACHE = os.path.expanduser('~/.gz/fuel/fuel.gazebosim.org/openrobotics/models')


# --------------------------------------------------------------------------- models
def prepare_base_model(fuel_name, local):
    dest = os.path.join(MODELS, local)
    if os.path.exists(os.path.join(dest, 'model.sdf')):
        return
    src_root = os.path.join(FUEL_CACHE, fuel_name.lower())
    if not os.path.isdir(src_root):
        print(f'  downloading {fuel_name} from Fuel')
        subprocess.run(['gz', 'fuel', 'download', '-u',
                        f'https://fuel.gazebosim.org/1.0/OpenRobotics/models/{fuel_name}'], check=True)
    version = sorted(os.listdir(src_root), key=int)[-1]
    src = os.path.join(src_root, version)
    print(f'  copying {src} -> models/{local}')
    shutil.copytree(src, dest, ignore=shutil.ignore_patterns('thumbnails'))
    sdf = open(os.path.join(dest, 'model.sdf')).read()
    sdf = re.sub(r'https://fuel\.gazebosim\.org/1\.0/openrobotics/models/[^/]+/\d+/files/', f'model://{local}/', sdf)
    sdf = sdf.replace(f'<model name="{fuel_name}">', f'<model name="{local}">')
    with open(os.path.join(dest, 'model.sdf'), 'w') as f:
        f.write(sdf)
    cfg = open(os.path.join(dest, 'model.config')).read()
    cfg = cfg.replace(f'<name>{fuel_name}</name>', f'<name>{local}</name>')
    cfg = cfg.replace('</description>', f'    Copied from https://app.gazebosim.org/OpenRobotics/fuel/models/{fuel_name} '
                      f'(version {version}) by scripts/gen_trees.py; URIs rewritten to model://.\n  </description>')
    with open(os.path.join(dest, 'model.config'), 'w') as f:
        f.write(cfg)


def write_wrapper(species, cls):
    fuel_name, local, natural_h = SPECIES[species]
    scale = CLASSES[cls] / natural_h
    name = f'tree_{species}_{cls}'
    d = os.path.join(MODELS, name)
    os.makedirs(d, exist_ok=True)
    base = open(os.path.join(MODELS, local, 'model.sdf')).read()
    visuals = re.findall(r'<visual .*?</visual>', base, re.S)
    body = ''
    for v in visuals:
        v = re.sub(r'(<mesh>)', f'\\1\n                        <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>', v, count=1)
        v = re.sub(r'\s*<script>.*?</script>', '', v, flags=re.S)   # legacy Ogre scripts: gz-sim warns, <pbr> is enough
        body += '            ' + v.strip() + '\n'
    with open(os.path.join(d, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<!-- Generated by scripts/gen_trees.py: {local} scaled x{scale:.2f} to ~{CLASSES[cls]:.0f} m, no collision. -->
<sdf version="1.7">
    <model name="{name}">
        <static>true</static>
        <link name="link">
{body}        </link>
    </model>
</sdf>
''')
    with open(os.path.join(d, 'model.config'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <description>{fuel_name} scaled to about {CLASSES[cls]:.0f} m (generated by scripts/gen_trees.py).</description>
</model>
''')
    return name


# --------------------------------------------------------------------------- candidates
class Grid:
    """DEM-aligned rasters: terrain z, lake mask, canopy height, local x/y."""

    def __init__(self):
        with open(os.path.join(DATA, 'bbox.json')) as f:
            self.bbox = json.load(f)
        g = np.load(os.path.join(DATA, 'terrain_grid.npz'))
        self.z, self.mask, self.xs, self.ys = g['z'], g['mask'], g['xs'], g['ys']
        self.n = self.z.shape[0]
        dtm = np.fromfile(os.path.join(DATA, 'dtm.bin'), dtype='<f4').reshape(self.n, self.n)
        mns = np.fromfile(os.path.join(DATA, 'mns.bin'), dtype='<f4').reshape(self.n, self.n)
        self.canopy = np.clip(mns - dtm, 0, None)
        self.conv = geo.make_converter()
        self.px_m = abs(self.xs[1] - self.xs[0])

    def rasterize(self, rings):
        """Boolean raster of lat/lon rings [(role, geometry), ...] (outer first)."""
        img = Image.new('L', (self.n, self.n), 0)
        dr = ImageDraw.Draw(img)
        for role, geometry in sorted(rings, key=lambda r: r[0] != 'outer'):
            poly = [self.px(g['lat'], g['lon'])[::-1] for g in geometry]  # (c, r)
            if len(poly) >= 3:
                dr.polygon(poly, fill=255 if role == 'outer' else 0)
        return np.array(img) > 0

    def px(self, lat, lon):
        b = self.bbox
        c = (lon - b['lon_w']) / (b['lon_e'] - b['lon_w']) * self.n
        r = (b['lat_n'] - lat) / (b['lat_n'] - b['lat_s']) * self.n
        return r, c

    def inside(self, r, c):
        return 0 <= r < self.n and 0 <= c < self.n

    def sample(self, arr, r, c, radius=1):
        r0, r1 = max(0, int(r) - radius), min(self.n, int(r) + radius + 1)
        c0, c1 = max(0, int(c) - radius), min(self.n, int(c) + radius + 1)
        return arr[r0:r1, c0:c1]

    def local(self, lat, lon):
        x, y, _ = self.conv.local_from_latlon(lat, lon)
        return x, y


def species_of(tags):
    t = ' '.join(str(v) for v in tags.values()).lower()
    if tags.get('leaf_type') == 'needleleaved' or any(k in t for k in ('pinus', 'picea', 'abies', 'conifer')):
        return 'pine'
    return 'oak'


def walk(geometry, spacing):
    """Points every `spacing` metres along a lat/lon polyline."""
    pts = [(g['lat'], g['lon']) for g in geometry]
    out, carry = [], 0.0
    for (la1, lo1), (la2, lo2) in zip(pts, pts[1:]):
        dy = (la2 - la1) * 111320.0
        dx = (lo2 - lo1) * 111320.0 * math.cos(math.radians(la1))
        seg = math.hypot(dx, dy)
        d = carry
        while d <= seg:
            t = d / seg if seg else 0.0
            out.append((la1 + t * (la2 - la1), lo1 + t * (lo2 - lo1)))
            d += spacing
        carry = d - seg
    return out


def mask_fill(grid, m, rng):
    """Jittered grid of lat/lon points inside boolean raster m where there is canopy."""
    step = FILL_SPACING_M / grid.px_m
    out = []
    for r in np.arange(step / 2, grid.n, step):
        for c in np.arange(step / 2, grid.n, step):
            rr, cc = r + rng.uniform(-0.4, 0.4) * step, c + rng.uniform(-0.4, 0.4) * step
            if grid.inside(rr, cc) and m[int(rr), int(cc)] and grid.canopy[int(rr), int(cc)] >= MIN_CANOPY_M:
                b = grid.bbox
                out.append((b['lat_n'] - rr / grid.n * (b['lat_n'] - b['lat_s']),
                            b['lon_w'] + cc / grid.n * (b['lon_e'] - b['lon_w'])))
    return out


def rings_of(e):
    if e['type'] == 'way':
        return [('outer', e['geometry'])]
    return [(m['role'], m['geometry']) for m in e.get('members', []) if 'geometry' in m]


def candidates(grid, rng):
    from scipy import ndimage
    with open(os.path.join(DATA, 'trees.json')) as f:
        elements = json.load(f)['elements']
    out = []  # (lat, lon, species, source)
    buildings = np.zeros_like(grid.mask)
    for e in elements:
        tags = e.get('tags', {})
        sp = species_of(tags)
        if 'building' in tags:
            buildings |= grid.rasterize(rings_of(e))
        elif e['type'] == 'node':
            out.append((e['lat'], e['lon'], sp, 'tree'))
        elif e['type'] == 'way' and tags.get('natural') == 'tree_row':
            out += [(la, lo, sp, 'row') for la, lo in walk(e['geometry'], ROW_SPACING_M)]
        else:
            out += [(la, lo, sp, 'wood') for la, lo in mask_fill(grid, grid.rasterize(rings_of(e)), rng)]
    # lakeside band straight from the canopy model
    it = int(round(SHORE_BAND_M / grid.px_m))
    band = ndimage.binary_dilation(grid.mask, iterations=it) & ~grid.mask
    band &= ~ndimage.binary_dilation(buildings, iterations=int(round(BUILDING_MARGIN_M / grid.px_m)))
    out += [(la, lo, 'oak', 'shore') for la, lo in mask_fill(grid, band, rng)]
    return out


def place(grid, cands, rng):
    trees = []
    for lat, lon, sp, src in cands:
        r, c = grid.px(lat, lon)
        if not grid.inside(r, c) or grid.mask[int(r), int(c)]:
            continue
        x, y = grid.local(lat, lon)
        if math.hypot(x, y) > MAX_RADIUS_M:
            continue
        h = float(grid.sample(grid.canopy, r, c).max())
        if src == 'tree' and h < 2.0:
            h = 7.0                                   # OSM says tree, DEM does not: default size
        cls = 's' if h < 7.0 else ('m' if h < 12.0 else 'l')
        z = float(grid.sample(grid.z, r, c).min()) - 0.05
        trees.append((math.hypot(x, y), x, y, z, sp, cls, src))
    # de-duplicate (OSM nodes inside filled polygons), nearest first, cap
    trees.sort()
    kept = []
    for t in trees:
        if all(math.hypot(t[1] - k[1], t[2] - k[2]) > 3.0 for k in kept[-60:]):
            kept.append(t)
        if len(kept) >= TREE_CAP:
            break
    return kept


# --------------------------------------------------------------------------- world file
def splice(trees, rng):
    world = open(WORLD).read()
    i, j = world.index(BEGIN), world.index(END)
    old_block = world[i + len(BEGIN):j]
    manual = [ln for ln in old_block.splitlines() if '<!-- manual -->' in ln]
    lines = []
    for k, (_, x, y, z, sp, cls, src) in enumerate(trees):
        yaw = rng.uniform(0, 2 * math.pi)
        lines.append(f'    <include><name>tree_{k:03d}</name><uri>model://tree_{sp}_{cls}</uri>'
                     f'<pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.2f}</pose></include>  <!-- {src} -->')
    block = '\n' + '\n'.join(lines + manual) + '\n    '
    with open(WORLD, 'w') as f:
        f.write(world[:i + len(BEGIN)] + block + world[j:])
    return len(manual)


def preview(grid, trees, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    Image.MAX_IMAGE_PIXELS = None
    ortho = Image.open(os.path.join(DATA, 'ortho.jpg')).convert('RGB').resize((2048, 2048))
    ext = [grid.xs[0], grid.xs[-1], grid.ys[-1], grid.ys[0]]
    fig, ax = plt.subplots(figsize=(9, 9), dpi=110)
    ax.imshow(ortho, extent=ext)
    colors = {'tree': 'yellow', 'row': 'orange', 'wood': 'lime', 'shore': 'magenta'}
    for src in colors:
        pts = [(t[1], t[2], t[5]) for t in trees if t[6] == src]
        if pts:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=[{'s': 6, 'm': 12, 'l': 20}[p[2]] for p in pts],
                       c=colors[src], edgecolors='k', linewidths=0.3, label=f'{src} ({len(pts)})')
    ax.contour(grid.xs, grid.ys, grid.mask.astype(float), levels=[0.5], colors='cyan', linewidths=0.8)
    ax.set_xlim(-MAX_RADIUS_M, MAX_RADIUS_M); ax.set_ylim(-MAX_RADIUS_M, MAX_RADIUS_M)
    ax.legend(loc='lower left'); ax.set_xlabel('x east [m]'); ax.set_ylabel('y north [m]')
    ax.set_title('trees placed by gen_trees.py (marker size = class)')
    fig.savefig(path, bbox_inches='tight'); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cap', type=int, default=None, help=f'maximum number of trees (default {TREE_CAP})')
    ap.add_argument('--dry-run', action='store_true', help='report only, do not touch the world file')
    args = ap.parse_args()
    if args.cap is not None:
        globals()['TREE_CAP'] = args.cap
    rng = np.random.default_rng(SEED)

    print('models:')
    for fuel_name, local, _ in SPECIES.values():
        prepare_base_model(fuel_name, local)
    for sp in SPECIES:
        for cls in CLASSES:
            write_wrapper(sp, cls)

    grid = Grid()
    cands = candidates(grid, rng)
    trees = place(grid, cands, rng)
    by_src = {s: sum(1 for t in trees if t[6] == s) for s in ('tree', 'row', 'wood', 'shore')}
    by_cls = {c: sum(1 for t in trees if t[5] == c) for c in CLASSES}
    print(f'{len(cands)} candidates -> {len(trees)} trees {by_src}, size classes {by_cls}, '
          f'pines {sum(1 for t in trees if t[4] == "pine")}')
    preview(grid, trees, os.path.join(DATA, 'trees_preview.png'))
    if args.dry_run:
        return
    kept = splice(trees, rng)
    print(f'wrote {len(trees)} <include> lines into {os.path.relpath(WORLD)} (kept {kept} manual lines)')


if __name__ == '__main__':
    main()
