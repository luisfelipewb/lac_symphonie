# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from lac_symphonie import geo  # noqa: E402

GT_EUROPE = (49.1021349, 6.2151242)   # OSM way 157475622
GT_LOCAL = (-145.16, -154.37)         # gz.math7 LOCAL2 == pure WGS-84 ENU


def test_pure_gt_building_offset():
    conv = geo.PureENU(geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
    x, y, z = conv.local_from_latlon(*GT_EUROPE)
    assert abs(x - GT_LOCAL[0]) < 0.05 and abs(y - GT_LOCAL[1]) < 0.05
    assert abs(z) < 0.05  # same altitude as the origin -> on the tangent plane (minus curvature)


def test_pure_round_trip():
    conv = geo.PureENU(geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
    for x, y, z in [(0, 0, 0), (1000, -700, 5.0), (-300, 400, -2.0)]:
        lat, lon, alt = conv.latlon_from_local(x, y, z)
        x2, y2, z2 = conv.local_from_latlon(lat, lon, alt)
        assert math.dist((x, y, z), (x2, y2, z2)) < 1e-3


def test_gz_matches_pure():
    pytest.importorskip('gz.math7')
    gz = geo.GzENU(geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
    pure = geo.PureENU(geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
    for dlat, dlon in [(0.0, 0.0), (0.009, 0.0), (0.0, 0.014), (-0.006, -0.010)]:  # ~1 km
        lat, lon = geo.ORIGIN_LAT + dlat, geo.ORIGIN_LON + dlon
        a = gz.local_from_latlon(lat, lon, geo.ORIGIN_ALT + 3.0)
        b = pure.local_from_latlon(lat, lon, geo.ORIGIN_ALT + 3.0)
        assert math.dist(a, b) < 0.01, (a, b)
        la, lo, al = gz.latlon_from_local(*a)
        assert abs(la - lat) < 1e-7 and abs(lo - lon) < 1e-7 and abs(al - geo.ORIGIN_ALT - 3.0) < 0.01


def test_read_origin_from_sdf():
    path = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'lac_symphonie.sdf')
    lat, lon, alt = geo.read_origin_from_sdf(path)
    assert (lat, lon, alt) == (geo.ORIGIN_LAT, geo.ORIGIN_LON, geo.ORIGIN_ALT)
