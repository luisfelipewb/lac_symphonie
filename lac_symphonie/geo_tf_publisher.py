# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Publish lat/lon points as TF frames under the world frame.

The world origin comes from the ``<spherical_coordinates>`` of the world SDF
(parameter ``world_sdf``) or from ``origin_*`` parameters.  Points come from a
YAML file (``points_file``) and, at runtime, from ``geographic_msgs/GeoPointStamped``
messages on ``~/add_point`` (``header.frame_id`` is the child frame name).

YAML format:
    points:
      - {name: georgia_tech, latitude: 49.1021349, longitude: 6.2151242}
      - {name: island, latitude: 49.10335, longitude: 6.21715, altitude: 192.0, yaw_deg: 90}

``altitude`` is absolute (WGS-84 metres, like the navsat fix); when omitted the
frame sits at the origin altitude, i.e. on the water plane (z = 0).
"""

import math

import rclpy
import yaml
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from lac_symphonie import geo


class GeoTfPublisher(Node):

    def __init__(self):
        super().__init__('geo_tf_publisher')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('world_sdf', '')
        self.declare_parameter('origin_latitude', geo.ORIGIN_LAT)
        self.declare_parameter('origin_longitude', geo.ORIGIN_LON)
        self.declare_parameter('origin_altitude', geo.ORIGIN_ALT)
        self.declare_parameter('points_file', '')

        self.world_frame = self.get_parameter('world_frame').value
        world_sdf = self.get_parameter('world_sdf').value
        if world_sdf:
            lat0, lon0, alt0 = geo.read_origin_from_sdf(world_sdf)
            self.get_logger().info(f'origin from {world_sdf}')
        else:
            lat0 = self.get_parameter('origin_latitude').value
            lon0 = self.get_parameter('origin_longitude').value
            alt0 = self.get_parameter('origin_altitude').value
        self.conv = geo.make_converter(lat0, lon0, alt0)
        self.get_logger().info(f'origin lat {lat0:.7f} lon {lon0:.7f} alt {alt0:.2f} '
                               f'({type(self.conv).__name__}), world frame "{self.world_frame}"')

        self.points = {}          # name -> (lat, lon, alt or None, yaw_deg)
        self.broadcaster = StaticTransformBroadcaster(self)
        points_file = self.get_parameter('points_file').value
        if points_file:
            self.load_points(points_file)
        self.publish_all()

        self.create_subscription(GeoPointStamped, '~/add_point', self.on_add_point, 10)

    def load_points(self, path):
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        for p in cfg.get('points', []):
            self.points[str(p['name'])] = (float(p['latitude']), float(p['longitude']),
                                           None if p.get('altitude') is None else float(p['altitude']),
                                           float(p.get('yaw_deg', 0.0)))
        self.get_logger().info(f'{len(self.points)} point(s) from {path}')

    def on_add_point(self, msg):
        name = msg.header.frame_id or f'geo_point_{len(self.points)}'
        alt = None if math.isnan(msg.position.altitude) else msg.position.altitude
        self.points[name] = (msg.position.latitude, msg.position.longitude, alt, 0.0)
        self.publish_all()

    def transform(self, name, lat, lon, alt, yaw_deg):
        x, y, z = self.conv.local_from_latlon(lat, lon, alt)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = name
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.z = math.sin(math.radians(yaw_deg) / 2.0)
        t.transform.rotation.w = math.cos(math.radians(yaw_deg) / 2.0)
        return t

    def publish_all(self):
        # a static broadcaster latches one message: always resend the full set
        tfs = [self.transform(n, *p) for n, p in self.points.items()]
        if tfs:
            self.broadcaster.sendTransform(tfs)
        for t in tfs:
            tr = t.transform.translation
            self.get_logger().info(f'{self.world_frame} -> {t.child_frame_id}: '
                                   f'x {tr.x:.2f} y {tr.y:.2f} z {tr.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GeoTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
