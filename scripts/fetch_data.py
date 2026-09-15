#!/usr/bin/env python3
# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Download the raw data for the Lac Symphonie world into data/ (git-ignored).

  data/lake.json   OSM relation 8841690 (shoreline + island), Overpass API
  data/trees.json  OSM trees / tree rows / woods / buildings inside the world bbox
  data/dtm.bin     IGN RGE ALTI 1 m DTM, float32 little-endian, row-major, north up
  data/mns.bin     IGN surface model (trees, buildings) at the same grid
  data/ortho.jpg   IGN orthophoto (~0.2 m) for the terrain texture

All endpoints are public and need no key.  Re-run to refresh; existing files
are skipped unless --force is given.
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

# World bbox (WGS-84), lake centred: ~885 m N-S x 877 m E-W.
LAT_S, LAT_N = 49.0995, 49.1075
LON_W, LON_E = 6.2115, 6.2235
DEM_PX = 1024          # ~0.86 m/px, native RGE ALTI is 1 m
ORTHO_PX = 4096        # ~0.21 m/px

LAKE_RELATION = 8841690
OVERPASS = ['https://overpass-api.de/api/interpreter',
            'https://overpass.kumi.systems/api/interpreter']
WMS = 'https://data.geopf.fr/wms-r/wms'

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')


def bbox_str():
    return f'{LAT_S},{LON_W},{LAT_N},{LON_E}'


def fetch(url, dest, data=None, force=False):
    if os.path.exists(dest) and not force:
        print(f'  keep   {os.path.relpath(dest)}')
        return
    print(f'  fetch  {os.path.relpath(dest)}')
    req = urllib.request.Request(url, data=data, headers={'User-Agent': 'lac_symphonie/0.1 (VRX world generator)'})
    with urllib.request.urlopen(req, timeout=180) as r:
        body = r.read()
    with open(dest, 'wb') as f:
        f.write(body)


def overpass(query, dest, force):
    # public servers answer 429/504 when busy: rotate mirrors, retry a few times
    data = urllib.parse.urlencode({'data': query}).encode()
    for attempt in range(6):
        url = OVERPASS[attempt % len(OVERPASS)]
        try:
            fetch(url, dest, data=data, force=force)
            break
        except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError) as e:
            code = getattr(e, 'code', None)
            if (code is not None and code not in (429, 502, 503, 504)) or attempt == 5:
                raise
            print(f'         {url}: {code or type(e).__name__}, retrying in 15 s')
            time.sleep(15)
    with open(dest) as f:
        n = len(json.load(f).get('elements', []))
    print(f'         {n} elements')


def wms(layer, fmt, px, dest, force):
    q = {'SERVICE': 'WMS', 'VERSION': '1.3.0', 'REQUEST': 'GetMap', 'STYLES': '',
         'LAYERS': layer, 'FORMAT': fmt, 'CRS': 'EPSG:4326',
         'BBOX': f'{LAT_S},{LON_W},{LAT_N},{LON_E}', 'WIDTH': px, 'HEIGHT': px}
    fetch(WMS + '?' + urllib.parse.urlencode(q), dest, force=force)
    size = os.path.getsize(dest)
    if fmt.startswith('image/x-bil') and size != px * px * 4:
        sys.exit(f'{dest}: got {size} bytes, expected {px * px * 4} (WMS error? inspect the file)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--force', action='store_true', help='re-download existing files')
    args = ap.parse_args()
    os.makedirs(DATA_DIR, exist_ok=True)
    print(f'bbox lat {LAT_S}..{LAT_N} lon {LON_W}..{LON_E}')

    overpass(f'[out:json][timeout:90];relation({LAKE_RELATION});out body geom;',
             os.path.join(DATA_DIR, 'lake.json'), args.force)
    overpass('[out:json][timeout:120];('
             f'node["natural"="tree"]({bbox_str()});'
             f'way["natural"="tree_row"]({bbox_str()});'
             f'way["natural"="wood"]({bbox_str()});'
             f'way["landuse"="forest"]({bbox_str()});'
             f'relation["natural"="wood"]({bbox_str()});'
             f'relation["landuse"="forest"]({bbox_str()});'
             f'way["building"]({bbox_str()});'
             f'relation["building"]({bbox_str()});'
             ');out body geom;',
             os.path.join(DATA_DIR, 'trees.json'), args.force)
    wms('ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES', 'image/x-bil;bits=32', DEM_PX,
        os.path.join(DATA_DIR, 'dtm.bin'), args.force)
    wms('ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES.MNS', 'image/x-bil;bits=32', DEM_PX,
        os.path.join(DATA_DIR, 'mns.bin'), args.force)
    wms('ORTHOIMAGERY.ORTHOPHOTOS', 'image/jpeg', ORTHO_PX,
        os.path.join(DATA_DIR, 'ortho.jpg'), args.force)
    with open(os.path.join(DATA_DIR, 'bbox.json'), 'w') as f:
        json.dump({'lat_s': LAT_S, 'lat_n': LAT_N, 'lon_w': LON_W, 'lon_e': LON_E,
                   'dem_px': DEM_PX, 'ortho_px': ORTHO_PX}, f, indent=2)
    print('done')


if __name__ == '__main__':
    main()
