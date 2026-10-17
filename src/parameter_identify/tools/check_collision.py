#!/usr/bin/env python3
"""O12 离线校验：用 python-pinocchio 复算一条激励轨迹记录里所有样本的
连杆间距离与连杆-地面净空，确认碰撞约束真的生效。

用法：
  python3 check_collision.py <urdf> <csv> [safe_distance] [ground_clearance]

退出码 0 = 所有样本满足约束；1 = 存在违例。

与控制器内实现的对应关系（O12）：
  - 碰撞对：去掉相邻连杆对（parentJoint 相差 <= 1）；地面平面 z=0，排除 base_link
    （安装座本就埋在地面下，仿真里是静态体、不产生接触）。
  - 判据：连杆间 距离 >= safe_distance；连杆-地面 网格最低点 >= ground_clearance。
  - 本脚本用**距离查询**（比控制器的 AABB 预筛 + 带裕度碰撞查询更精确），
    因此结果可以作为控制器实现的上界参考：控制器判可行 => 这里应该也全部满足。
"""
import sys

import numpy as np
import pinocchio as pin
import coal


def build_collision_model(urdf_path):
    model = pin.buildModelFromUrdf(urdf_path)
    geom = pin.buildGeomFromUrdf(model, urdf_path, pin.GeometryType.COLLISION)
    objs = geom.geometryObjects
    base_idx = [i for i, o in enumerate(objs) if "base_link" in o.name]
    base_idx = base_idx[0] if base_idx else None

    geom.removeAllCollisionPairs()
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            if abs(int(objs[i].parentJoint) - int(objs[j].parentJoint)) <= 1:
                continue
            geom.addCollisionPair(pin.CollisionPair(i, j))
    link_pairs = len(geom.collisionPairs)

    # 地面：局部 z=0 的无限大平面；placement 的平移 t_z 把平面放到世界 z=t_z。
    link_obj_count = len(geom.geometryObjects)
    ground = pin.GeometryObject("ground_plane", 0, pin.SE3.Identity(), coal.Plane(0.0, 0.0, 1.0, 0.0))
    gidx = geom.addGeometryObject(ground)
    for i in range(link_obj_count):
        if base_idx is not None and i == base_idx:
            continue
        geom.addCollisionPair(pin.CollisionPair(i, gidx))
    return model, geom, link_pairs, gidx


def main():
    urdf = sys.argv[1]
    csv_path = sys.argv[2]
    safe = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
    clearance = float(sys.argv[4]) if len(sys.argv) > 4 else 0.01

    model, geom, link_pairs, gidx = build_collision_model(urdf)
    data = model.createData()
    gdata = geom.createData()

    with open(csv_path) as f:
        header = f.readline().strip().split(",")
        pos_cols = [i for i, h in enumerate(header) if h.startswith("pos_")]
        rows = [line.strip().split(",") for line in f if line.strip()]
    print(f"samples={len(rows)} link_pairs={link_pairs} ground_pairs={len(geom.collisionPairs) - link_pairs}"
          f" safe={safe} clearance={clearance}")

    names = [geom.geometryObjects[p.first].name + " <-> " + geom.geometryObjects[p.second].name
             for p in geom.collisionPairs]
    worst = [1e9] * len(geom.collisionPairs)
    link_viol = 0
    ground_viol = 0

    # 地面平面位于 z = clearance：平面本身在 z=0，placement 平移 +clearance。
    ground_placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, clearance]))

    for row in rows:
        q = np.array([float(row[c]) for c in pos_cols])
        geom.geometryObjects[gidx].placement = ground_placement
        pin.computeDistances(model, data, geom, gdata, q)
        for k in range(len(geom.collisionPairs)):
            d = float(gdata.distanceResults[k].min_distance)
            worst[k] = min(worst[k], d)
            if k < link_pairs:
                if d < safe:
                    link_viol += 1
            else:
                if d < 0.0:
                    ground_viol += 1

    print("\n连杆间最小距离（越接近 0 越危险）：")
    for k in range(link_pairs):
        print(f"  {names[k]:40s} min={worst[k]:+.4f} m  {'< safe!' if worst[k] < safe else ''}")
    print("\n各连杆离地最小净空（地面位于 z=clearance，负值 = 额外净空不足）：")
    for k in range(link_pairs, len(geom.collisionPairs)):
        print(f"  {names[k]:40s} min={worst[k]:+.4f} m  {'< 0!' if worst[k] < 0 else ''}")

    print(f"\n违例样本数：连杆间(<{safe})={link_viol}，钻地(<{clearance})={ground_viol}")
    return 0 if (link_viol == 0 and ground_viol == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
