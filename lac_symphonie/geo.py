# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Geodetic (lat/lon/alt) <-> local ENU conversions for the Lac Symphonie world.

The world's ``<spherical_coordinates>`` block defines the origin of the Gazebo
world frame (= ROS ``map``).  The navsat sensor and the VRX scoring plugins use
``gz::math::SphericalCoordinates`` for the conversion; this module wraps the
Python binding of the same class (``gz.math7``, shipped with ROS Jazzy) so that
terrain, trees, spawn poses and TF frames all agree.  A pure-Python WGS-84
ECEF->ENU implementation is used when the binding is not importable (e.g. a
plain ``python3`` without the ROS environment sourced).
"""

import math

# Default origin: centroid of OSM relation 8841690 (Lac Symphonie) at the water
# level measured on the IGN RGE ALTI 1 m DTM.  Keep in sync with
# worlds/lac_symphonie.sdf (read_origin_from_sdf() is preferred at runtime).
ORIGIN_LAT = 49.1035229
ORIGIN_LON = 6.2171120
ORIGIN_ALT = 191.74

# WGS-84 ellipsoid
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def _geodetic_to_ecef(lat_deg, lon_deg, alt):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * sl * sl)
    return ((n + alt) * cl * math.cos(lon),
            (n + alt) * cl * math.sin(lon),
            (n * (1.0 - _E2) + alt) * sl)


def _ecef_to_geodetic(x, y, z):
    # Closed-form (Bowring) iteration, accurate to < 1 mm.
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    for _ in range(5):
        sl = math.sin(lat)
        n = _A / math.sqrt(1.0 - _E2 * sl * sl)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + alt)))
    sl = math.sin(lat)
    n = _A / math.sqrt(1.0 - _E2 * sl * sl)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


class PureENU:
    """WGS-84 geodetic <-> local ENU tangent plane, no external dependency."""

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        self._o = _geodetic_to_ecef(lat0, lon0, alt0)
        la, lo = math.radians(lat0), math.radians(lon0)
        sa, ca, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        # rows: east, north, up
        self._r = ((-so, co, 0.0),
                   (-sa * co, -sa * so, ca),
                   (ca * co, ca * so, sa))

    def local_from_latlon(self, lat, lon, alt=None):
        alt = self.alt0 if alt is None else alt
        d = [a - b for a, b in zip(_geodetic_to_ecef(lat, lon, alt), self._o)]
        return tuple(sum(r[i] * d[i] for i in range(3)) for r in self._r)

    def latlon_from_local(self, x, y, z=0.0):
        v = (x, y, z)
        ecef = [self._o[i] + sum(self._r[r][i] * v[r] for r in range(3)) for i in range(3)]
        return _ecef_to_geodetic(*ecef)


class GzENU:
    """Same interface, backed by gz.math7.SphericalCoordinates (EARTH_WGS84, ENU).

    Uses ``PositionTransform`` with the ``LOCAL2`` frame — the frame gz-sim
    itself uses for the navsat sensor.  The older ``LOCAL`` frame (and the
    ``local_from_spherical_position`` convenience call) carries a known
    gz-math 7 bug: ~2 % scale error and a sign flip in the inverse.
    """

    def __init__(self, lat0, lon0, alt0):
        from gz.math7 import Angle, SphericalCoordinates, Vector3d
        self._v3 = Vector3d
        self._ct = SphericalCoordinates.CoordinateType
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        sc = SphericalCoordinates(SphericalCoordinates.SurfaceType.EARTH_WGS84)
        sc.set_latitude_reference(Angle(math.radians(lat0)))
        sc.set_longitude_reference(Angle(math.radians(lon0)))
        sc.set_elevation_reference(alt0)
        sc.set_heading_offset(Angle(0.0))
        self._sc = sc

    def local_from_latlon(self, lat, lon, alt=None):
        alt = self.alt0 if alt is None else alt
        sph = self._v3(math.radians(lat), math.radians(lon), alt)
        p = self._sc.position_transform(sph, self._ct.SPHERICAL, self._ct.LOCAL2)
        return (p.x(), p.y(), p.z())

    def latlon_from_local(self, x, y, z=0.0):
        p = self._sc.position_transform(self._v3(x, y, z), self._ct.LOCAL2, self._ct.SPHERICAL)
        return (math.degrees(p.x()), math.degrees(p.y()), p.z())


def make_converter(lat0=ORIGIN_LAT, lon0=ORIGIN_LON, alt0=ORIGIN_ALT, prefer_gz=True):
    """Return a converter with ``local_from_latlon`` / ``latlon_from_local``."""
    if prefer_gz:
        try:
            return GzENU(lat0, lon0, alt0)
        except ImportError:
            pass
    return PureENU(lat0, lon0, alt0)


def read_origin_from_sdf(path):
    """Return (lat, lon, alt) from the ``<spherical_coordinates>`` of a world file."""
    import xml.etree.ElementTree as ET
    sc = ET.parse(path).getroot().find('.//spherical_coordinates')
    if sc is None:
        raise ValueError(f'{path}: no <spherical_coordinates> block found')

    def val(tag, default):
        e = sc.find(tag)
        return float(e.text) if e is not None and e.text else default

    return val('latitude_deg', 0.0), val('longitude_deg', 0.0), val('elevation', 0.0)
