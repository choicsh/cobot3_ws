"""Exclude mapped structure returns from motion tracking, never from braking."""
import math
import numpy as np


class StaticScanMask:
    def __init__(self, message, margin=.18):
        info = message.info
        self.resolution = info.resolution
        self.origin = (info.origin.position.x, info.origin.position.y)
        q = info.origin.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        occupied = np.asarray(message.data).reshape(info.height, info.width) >= 65
        self.mask = occupied.copy()
        # Unknown cells are not silently treated as known walls.
        radius = math.ceil(margin/info.resolution)
        padded = np.pad(occupied, radius)
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                if math.hypot(dx,dy)*info.resolution <= margin:
                    self.mask |= padded[radius+dy:radius+dy+info.height,
                                        radius+dx:radius+dx+info.width]

    def contains(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ix, iy = math.floor((c*dx+s*dy)/self.resolution), math.floor((-s*dx+c*dy)/self.resolution)
        return (0 <= iy < self.mask.shape[0] and 0 <= ix < self.mask.shape[1]
                and bool(self.mask[iy,ix]))
