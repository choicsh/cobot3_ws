"""ROS-free cost grid adapter; inputs follow ROS message field conventions."""
import math
import numpy as np
from nav_to_goal.hospital_avoidance import SafetySettings, wrap


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp_seconds(header):
    return header.stamp.sec+header.stamp.nanosec/1e9


class Grid:
    def __init__(self, message, raw=False):
        metadata = message.metadata if raw else message.info
        self.width = metadata.size_x if raw else metadata.width
        self.height = metadata.size_y if raw else metadata.height
        self.resolution = metadata.resolution
        self.origin = metadata.origin
        self.frame = message.header.frame_id
        values = np.asarray(message.data).reshape(self.height, self.width)
        self.blocked = ((values >= 253) if raw else ((values < 0) | (values >= 65)))
        self.stamp = stamp_seconds(message.header)

    def body_clear(self, pose, settings=SafetySettings(), require_inside=True):
        """Check every occupied cell in the rotated rectangle's bounding box."""
        origin_yaw = yaw_of(self.origin.orientation)
        dx, dy = pose[0]-self.origin.position.x, pose[1]-self.origin.position.y
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        x, y, yaw = c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-origin_yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x+c*a-s*b, y+s*a+c*b) for a in (-settings.rear, settings.front)
                   for b in (-settings.half_width, settings.half_width)]
        # Account for footprint padding and cells crossing polygon boundaries.
        pad = .01+self.resolution/math.sqrt(2)
        lo_x = math.floor((min(p[0] for p in corners)-pad)/self.resolution)
        hi_x = math.floor((max(p[0] for p in corners)+pad)/self.resolution)
        lo_y = math.floor((min(p[1] for p in corners)-pad)/self.resolution)
        hi_y = math.floor((max(p[1] for p in corners)+pad)/self.resolution)
        if require_inside and (lo_x < 0 or lo_y < 0 or hi_x >= self.width or hi_y >= self.height):
            return False
        lo_x, hi_x = max(0, lo_x), min(self.width-1, hi_x)
        lo_y, hi_y = max(0, lo_y), min(self.height-1, hi_y)
        if lo_x > hi_x or lo_y > hi_y:
            return not require_inside
        iy, ix = np.nonzero(self.blocked[lo_y:hi_y+1, lo_x:hi_x+1])
        dx = (ix+lo_x+.5)*self.resolution-x
        dy = (iy+lo_y+.5)*self.resolution-y
        bx, by = c*dx+s*dy, -s*dx+c*dy
        return not np.any((bx >= -settings.rear-pad) & (bx <= settings.front+pad) &
                          (np.abs(by) <= settings.half_width+pad))

