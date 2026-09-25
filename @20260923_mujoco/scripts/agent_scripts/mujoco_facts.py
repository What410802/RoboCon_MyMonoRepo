#!/usr/bin/env python3
"""导出写文档要用到的 MuJoCo 事实（geom 类型枚举、接触相关默认值等）。

只为给 docs 里的笔记提供**可复现的出处**，不是仿真脚本。
用法：pixi run python scripts/agent_scripts/mujoco_facts.py
"""

from __future__ import annotations

import mujoco

print(f"mujoco {mujoco.__version__}")

print("\n== mjtGeom 全部成员（XML geom 的 type 取这些枚举名的小写）==")
for name in sorted(dir(mujoco.mjtGeom)):
    if name.startswith("mjGEOM_"):
        member = getattr(mujoco.mjtGeom, name)
        print(f"  {member.name:<16} = {int(member):>3}   -> type=\"{member.name.removeprefix('mjGEOM_').lower()}\"")

print("\n== 哪些 type 真能在 XML 的 geom 里用（逐个试编译）==")
CANDIDATES = [
    "plane", "hfield", "sphere", "capsule", "ellipsoid", "cylinder", "box", "mesh", "sdf",
    "arrow", "arrow1", "arrow2", "line", "linebox", "flex", "skin", "label", "triangle", "none",
]
for name in CANDIDATES:
    xml = f'<mujoco><worldbody><geom type="{name}" size="0.1"/></worldbody></mujoco>'
    try:
        mujoco.MjModel.from_xml_string(xml)
        verdict = "OK"
    except Exception as exc:  # noqa: BLE001 - 探针，失败就是结论
        verdict = "NG: " + str(exc).splitlines()[0][:70]
    print(f"  {name:<10} {verdict}")

print("\n== 从「含一个 geom」的模型看接触相关默认值 ==")
model = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><geom size="0.1"/></worldbody></mujoco>'
)
for field in ("geom_condim", "geom_friction", "geom_solref", "geom_solimp", "geom_margin", "geom_gap"):
    print(f"  {field:<14} = {getattr(model, field)[0]}")
print(f"  opt.o_friction = {model.opt.o_friction}   (全局摩擦覆盖；MJCF 里写 <option o_friction=...>)")

print("\n== <contact><pair> 对应哪些模型字段 ==")
print("  ", [a for a in dir(model) if a.startswith("pair") and not a.endswith("__")])

print("\n== mujoco 里与 condim / friction 相关的枚举或常量 ==")
print("  ", [a for a in dir(mujoco) if a.startswith("mjt") and ("ondim" in a or "riction" in a)] or "（无，condim/friction 用普通整数与数组）")

print("\n== mjtGeom 里不能直接作为 XML type 的项 ==")
print("   mjGEOM_NONE =", int(mujoco.mjtGeom.mjGEOM_NONE), "(内部占位，XML 不能写)")

print("\n== <geom friction=...> 少给数字会怎样 ==")
for spec in ('friction="2"', 'friction="2 0.1"', 'friction="2 0.1 0.2"', 'friction="2 0.1 0.2 0.3"'):
    xml = f'<mujoco><worldbody><geom size="0.1" {spec}/></worldbody></mujoco>'
    try:
        probe = mujoco.MjModel.from_xml_string(xml)
        print(f"  {spec:<34} OK -> {probe.geom_friction[0]}")
    except Exception as exc:  # noqa: BLE001
        print(f"  {spec:<34} NG: {str(exc).splitlines()[0][:60]}")

print("\n== <geom condim=...> 合法取值 ==")
for value in (1, 2, 3, 4, 5, 6):
    xml = f'<mujoco><worldbody><geom size="0.1" condim="{value}"/></worldbody></mujoco>'
    try:
        probe = mujoco.MjModel.from_xml_string(xml)
        print(f"  condim={value} OK -> {probe.geom_condim[0]}")
    except Exception as exc:  # noqa: BLE001
        print(f"  condim={value} NG: {str(exc).splitlines()[0][:60]}")
