#!/usr/bin/env python3
"""从 arm_model 的固定基座模型生成「运动学基座（mocap）」MuJoCo 场景。

为什么需要单独一份模型：
  robotic_arm.xml 同时被现有 5 个 scene*.xml 使用，那些场景里的基座必须钉死在 world。

为什么用 mocap 而不是 freejoint：
  freejoint 的基座有质量，手臂反力会让它产生加速度；每步把 qpos/qvel 「钉回去」并不能
  消除这项加速度，手臂会看到一个被放大甚至反向的重力（离线实测初始 joint qacc 与固定
  基座相差 4 倍以上，控制器开重力前馈后会发散）。
  mocap 体没有自由度、没有加速度，手臂的关节动力学与固定基座模型逐位一致（离线对比：
  初始 joint qacc 完全相同），正好与控制器里的固定基座 Pinocchio 模型对齐。
  代价是基座速度不进入 MuJoCo 的科氏/离心项——本实验的基座速度下这项 < 0.3% g。

生成内容（输出到 --out-dir）：
  robotic_arm_mocap.xml   = robotic_arm.xml + base_link 的 mocap="true" + 基座抬高 base_z
  scene_base_motion.xml   = scene.xml，include 指向 robotic_arm_mocap.xml
  *.STL                   = 指向 --arm-model-dir 的符号链接（保持相对 mesh 路径不变）

用法：
  python3 generate_float_mjcf.py \
      --arm-model-dir $(ros2 pkg prefix arm_model)/share/arm_model/model \
      --out-dir /tmp/arm_base_motion
"""

import argparse
import os
import re
import sys

BASE_BODY_RE = re.compile(r'<body\s+name="base_link"([^>]*)>')
INCLUDE_RE = re.compile(r'<include\s+file="robotic_arm\.xml"\s*/>')
GEOM_BASE_RE = re.compile(r'(<geom\b[^>]*mesh="base_link"[^>]*)/>')
ASSET_SUFFIXES = (".stl", ".obj", ".msh", ".png", ".jpg")


def fail(message):
    print("generate_float_mjcf: " + message, file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Generate the mocap-base MuJoCo scene")
    parser.add_argument("--arm-model-dir", required=True, help="directory containing robotic_arm.xml / scene.xml")
    parser.add_argument("--out-dir", required=True, help="output directory (STL symlinks are created here)")
    parser.add_argument(
        "--base-z",
        type=float,
        default=0.06,
        help=(
            "lift the base by this many metres. 必须足够高：base_link/link1 的网格在基座原点"
            "以下约 34 mm，原始场景里它们本来就扎在地面里；基座一动，地面摩擦会直接拖住 joint1"
            "（实测关节滞后 57 mrad、末端误差 27 mm RMS）。0.06 留出约 26 mm 净空。"
        ),
    )
    args = parser.parse_args()

    model_dir = os.path.abspath(args.arm_model_dir)
    out_dir = os.path.abspath(args.out_dir)
    arm_path = os.path.join(model_dir, "robotic_arm.xml")
    scene_path = os.path.join(model_dir, "scene.xml")

    for path in (arm_path, scene_path):
        if not os.path.isfile(path):
            fail("missing input file: " + path)

    arm_text = open(arm_path, "r", encoding="utf-8").read()
    if 'mocap="true"' in arm_text:
        fail("robotic_arm.xml already contains a mocap body; refusing to mark base_link again")

    match = BASE_BODY_RE.search(arm_text)
    if match is None:
        fail("cannot find '<body name=\"base_link\">' in " + arm_path)
    attrs = match.group(1)
    if "pos=" in attrs:
        fail("base_link already has a 'pos' attribute; review the model before generating a mocap variant")

    open_tag = '<body name="base_link" pos="0 0 %.4f" mocap="true">' % args.base_z
    arm_mocap_text = arm_text[: match.start()] + open_tag + arm_text[match.end():]

    # 基座网格不参与碰撞：基座是运动学体，接触对它没有意义，只会给求解器添乱。
    geom_match = GEOM_BASE_RE.search(arm_mocap_text)
    if geom_match is not None and "contype=" not in geom_match.group(0):
        replacement = geom_match.group(1) + ' contype="0" conaffinity="0"/>'
        arm_mocap_text = arm_mocap_text[: geom_match.start()] + replacement + arm_mocap_text[geom_match.end():]

    scene_text = open(scene_path, "r", encoding="utf-8").read()
    scene_mocap_text, replaced = INCLUDE_RE.subn('<include file="robotic_arm_mocap.xml"/>', scene_text)
    if replaced != 1:
        fail("expected exactly one '<include file=\"robotic_arm.xml\"/>' in " + scene_path)

    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "robotic_arm_mocap.xml"), "w", encoding="utf-8") as handle:
        handle.write(arm_mocap_text)
    with open(os.path.join(out_dir, "scene_base_motion.xml"), "w", encoding="utf-8") as handle:
        handle.write(scene_mocap_text)

    # MuJoCo 按 MJCF 文件所在目录解析 mesh 的相对路径，因此把资产软链到输出目录。
    for name in sorted(os.listdir(model_dir)):
        if not name.lower().endswith(ASSET_SUFFIXES):
            continue
        target = os.path.join(out_dir, name)
        source = os.path.join(model_dir, name)
        if os.path.islink(target) or os.path.isfile(target):
            os.remove(target)
        os.symlink(source, target)

    print(os.path.join(out_dir, "scene_base_motion.xml"))


if __name__ == "__main__":
    main()
