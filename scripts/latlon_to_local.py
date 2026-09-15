#!/usr/bin/env python3
# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Convert lat/lon to local world x/y for the Lac Symphonie world, or back.

    latlon_to_local.py 49.1021349 6.2151242            -> x y z
    latlon_to_local.py --reverse -146.9 -158.2         -> lat lon alt
    latlon_to_local.py --world path/to/other.sdf LAT LON  (origin read from that file)
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from lac_symphonie import geo  # noqa: E402


def default_world():
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, '..', 'worlds', 'lac_symphonie.sdf'),
                 os.path.join(here, '..', 'share', 'lac_symphonie', 'worlds', 'lac_symphonie.sdf')):
        if os.path.exists(cand):
            return cand
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('a', type=float, help='latitude (deg) or, with --reverse, x (m)')
    ap.add_argument('b', type=float, help='longitude (deg) or, with --reverse, y (m)')
    ap.add_argument('c', type=float, nargs='?', default=None, help='altitude (m) or z (m)')
    ap.add_argument('--reverse', action='store_true', help='convert local x y [z] to lat lon alt')
    ap.add_argument('--world', default=default_world(), help='world SDF to read the origin from')
    ap.add_argument('--pure', action='store_true', help='force the pure-python converter')
    args = ap.parse_args()

    origin = geo.read_origin_from_sdf(args.world) if args.world else (geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
    conv = geo.make_converter(*origin, prefer_gz=not args.pure)
    if args.reverse:
        lat, lon, alt = conv.latlon_from_local(args.a, args.b, args.c or 0.0)
        print(f'lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}')
    else:
        x, y, z = conv.local_from_latlon(args.a, args.b, args.c)
        print(f'x={x:.2f} y={y:.2f} z={z:.2f}')


if __name__ == '__main__':
    main()
