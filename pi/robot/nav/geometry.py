"""复用旧凸包/SAT思想；旋转弯道以正式C变化率界覆盖采样间运动。"""

import math


def hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        return points

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])

    lower, upper = [], []
    for sequence, result in ((points, lower), (reversed(points), upper)):
        for point in sequence:
            while len(result) > 1 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
    return lower[:-1] + upper[:-1]


def rectangle(bounds):
    x0, y0, x1, y1 = bounds
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def bounds(points):
    return min(p[0] for p in points), min(p[1] for p in points), max(p[0] for p in points), max(p[1] for p in points)


def intersects(first, second):
    a, b = bounds(first), bounds(second)
    if a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1]:
        return False
    for polygon in (first, second):
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            aa = [x*axis[0]+y*axis[1] for x, y in first]
            bb = [x*axis[0]+y*axis[1] for x, y in second]
            if max(aa) < min(bb) - 1e-12 or max(bb) < min(aa) - 1e-12:
                return False
    return True


def footprint(pose, body, margin=0, movement=0):
    co, si = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    corners = [(pose.x_m + co*x - si*y, pose.y_m + si*x + co*y)
               for x, y in ((-body['rear_m']-margin, -body['right_m']-margin),
                            (body['front_m']+margin, -body['right_m']-margin),
                            (body['front_m']+margin, body['left_m']+margin),
                            (-body['rear_m']-margin, body['left_m']+margin))]
    # 世界轴方形包含同半径圆盘；覆盖任意方向位移和转动角造成的角点位移。
    return hull([(x+dx, y+dy) for x, y in corners
                 for dx, dy in ((-movement, -movement), (movement, -movement),
                                (movement, movement), (-movement, movement))])


def simplify(points, tolerance=.0005):
    """RDP只简化显示点，不修改控制/碰撞证据；完整原始序列由模型有界保留。"""
    if len(points) <= 2:
        return list(points)
    keep = {0, len(points)-1}
    pending = [(0, len(points)-1)]
    while pending:
        first, last = pending.pop()
        a, b = points[first], points[last]
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        best, selected = tolerance*tolerance, None
        for index in range(first+1, last):
            x, y = points[index][0]-a[0], points[index][1]-a[1]
            ratio = max(0, min(1, (x*dx+y*dy)/length2)) if length2 else 0
            distance = (x-ratio*dx)**2+(y-ratio*dy)**2
            if distance > best:
                best, selected = distance, index
        if selected is not None:
            keep.add(selected)
            pending.extend(((first,selected),(selected,last)))
    return [points[i] for i in sorted(keep)]
