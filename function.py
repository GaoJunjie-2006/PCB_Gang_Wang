# -*- coding: utf-8 -*-
"""
function.py —— PCB 文件 -> 钢网(Stencil) + 治具(Jig) 技术验证版
================================================================

设计目标（对应需求）
--------------------
1. 通用的输入格式：**Gerber RS-274X + Excellon 钻孔**。
   Altium / 立创EDA(LCEDA) / KiCad 等所有 EDA 都能导出，是事实标准。
2. 插件孔（2.54 排针、安装孔等打穿孔）**不允许开成钢网**：
   用钻孔文件生成"禁布区"，与钢网开孔做布尔运算，整孔删除或裁剪。
3. 输出 = 按板框生成的钢网 + 外圈一圈治具，**总有一个面是平的**
   （治具顶端与钢网顶端共面），方便 3D 打印机打印。
4. 简单前端：设置钢网厚度、板子厚度等；简单预览（俯视 + 剖面 + 3D HTML）。

几何结构（"使用姿态"，Z 向上，桌面 z=0）
-----------------------------------------
        z = tb+ts  ┌────────────────────────────┐  <- 治具顶面 与 钢网顶面 **共面**
                   │        治具环 (Jig)         │
        z = tb     ├──────────┬─────────────────┤  <- 钢网底面（贴住板子顶面）
                   │  钢网    │   板子空腔       │
                   │ (开孔)   │  (板子从下方放入) │     空腔深度 = 板厚 tb
        z = 0      └──────────┴─────────────────┘  <- 桌面

为了好打印，导出时默认整体翻转 180°（钢网面朝下贴热床）：
   翻转后底面 = 钢网底面 + 治具底面（同一平面，整面贴热床，不打桥）
   翻转后由下往上：0~ts 整块底板（含开孔），ts~tb+ts 只有外圈墙
   => 全程无悬空、无桥接，且上下两个大面都是绝对平面。

依赖：shapely（必需）、tkinter（GUI，标准库）。不依赖 OpenCASCADE。
      坐标解析、布尔运算、三角化、STL/STEP 写出全部自己实现，方便后续移植 C++。

用法：
    python function.py                     # 打开 GUI
    python function.py --selftest          # 生成合成样例并跑通全流程（自检）
    python function.py --input <目录或zip> --out <输出目录> [更多参数]

打包成 exe（单文件、免装 Python，车间直接双击用）：
    conda activate mess_3.11
    pip install pyinstaller
    set BIN=D:\\APPS\\ENV\\envs\\mess_3.11\\Library\\bin
    pyinstaller --noconfirm --onefile --console --name 钢网治具生成器 ^
        --distpath dist --workpath build --specpath build ^
        --add-binary "%BIN%\\tcl86t.dll;." --add-binary "%BIN%\\tk86t.dll;." ^
        --add-binary "%BIN%\\zlib1.dll;." --add-binary "%BIN%\\ffi-8.dll;." ^
        function.py
  产物 dist\\钢网治具生成器.exe（约 25 MB）：
    · 双击          -> 开 GUI（黑框自动隐藏）
    · 带参数命令行  -> 正常跑批处理，日志留在控制台
  那四个 --add-binary 是必须的：conda 把 tcl/tk 的 dll 放在 Library\\bin，
  PyInstaller 扫不到，不补的话打出来的 exe 一开界面就 "DLL load failed"。
  注意：exe 是把本文件连同 Python、shapely 一起打包，代码仍然只有这一个文件。
"""

from __future__ import annotations

import argparse
import base64
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import time
import traceback
import zipfile
from dataclasses import dataclass, field, fields

# ----------------------------------------------------------------------------
# shapely
# ----------------------------------------------------------------------------
try:
    import shapely
    from shapely import ops as shops
    from shapely.geometry import (
        GeometryCollection, LineString, MultiLineString,
        MultiPolygon, Point, Polygon, box,
    )
    from shapely.geometry.polygon import orient
    from shapely.ops import polygonize
except Exception as exc:  # pragma: no cover
    sys.stderr.write(
        "缺少 shapely，请先安装：\n"
        "    conda activate mess_3.11\n"
        "    pip install shapely\n"
        f"原始错误: {exc}\n")
    raise


# ============================================================================
# 0. 日志
# ============================================================================
APP_TITLE = "PCB 钢网 + 治具 生成器"
APP_VERSION = "1.1.1"          # 跟 git tag 同步，报 bug 时报的就是这个号

LOG: list[str] = []


def setup_console() -> None:
    """输出被重定向（管道/文件）时统一按 UTF-8 写，避免中文和 '²' 这类字符崩掉。

    Python 直接连着控制台时走的是控制台 API（内部 UTF-16），中文本来就没问题；
    可一旦 stdout 变成管道或文件，它就退回系统 ANSI 代码页——中文 Windows 上是
    GBK，打印 "mm²" 会直接 UnicodeEncodeError，把整个导出过程打断。打包成 exe
    之后（车间里免不了 `xx.exe -i ... > log.txt`）非常容易撞上，所以启动时收拾一遍。
    errors="replace" 是兜底：万一还有编不出来的字符，显示成 '?' 也绝不抛异常。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            if stream is not None and not stream.isatty():
                stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def log(msg: str = "") -> None:
    LOG.append(str(msg))
    try:
        print(msg, flush=True)
    except UnicodeEncodeError:
        # 极端情况下（比如日志被重定向到别的编码）宁可少几个字，也不能让活儿断掉
        print(msg.encode("ascii", "replace").decode("ascii"), flush=True)


def warn(msg: str) -> None:
    log("  [警告] " + msg)


class JobError(Exception):
    """业务流程可预期的错误（给用户看的）"""


# ============================================================================
# 1. 参数
# ============================================================================
@dataclass
class Params:
    # --- 尺寸（mm）---
    stencil_thickness: float = 0.15   # 钢网厚度
    board_thickness: float = 1.6      # 板厚（决定治具空腔深度）
    jig_wall: float = 5.0             # 治具环壁厚
    board_clearance: float = 0.20     # 板框与治具空腔的单边间隙
    # --- 开孔规则 ---
    hole_clearance: float = 0.15      # 打穿孔禁布区外扩（半径方向）
    hole_policy: str = "clip"         # clip=只挖掉孔位（默认）/ drop=整孔删除
    via_dia: float = 0.40             # ≤该直径的孔算过孔，不参与避让：
                                      # 盘中孔（焊盘里有散热过孔）的焊盘要保住，
                                      # 不能因为焊盘里穿了过孔就把整个焊盘删掉
    min_aperture: float = 0.15        # 小于该宽度的开孔直接剔除（0=不过滤）
    aperture_offset: float = 0.0      # 开孔补偿，正=开孔变大（FDM 打孔偏小可填 0.05~0.1）
    # --- 精度 / 输出 ---
    arc_tolerance: float = 0.01       # 圆弧离散弦高误差（mm）
    layer: str = "top"                # top / bottom 钢网
    include_jig: bool = True          # 是否带治具环
    flip_for_print: bool = True       # 导出时翻转成"钢网朝下"的打印姿态
    make_html: bool = True            # 额外生成 3D 预览 HTML
    out_dir: str = ""


# ============================================================================
# 2. 几何小工具
# ============================================================================
EMPTY = GeometryCollection()


def polys_of(geom) -> list[Polygon]:
    """把任意几何拆成 Polygon 列表"""
    if geom is None or geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, (MultiPolygon, GeometryCollection)):
        out = []
        for g in geom.geoms:
            out.extend(polys_of(g))
        return out
    return []


def U(geoms, grid=None):
    """union_all，兼容不同 shapely 版本"""
    gs = [g for g in geoms if g is not None and not g.is_empty]
    if not gs:
        return EMPTY
    if len(gs) == 1:
        return gs[0]
    try:
        return shapely.union_all(gs, grid_size=grid)
    except Exception:
        return shops.unary_union(gs)


def D(a, b):
    """difference，空值安全"""
    if a is None or a.is_empty:
        return EMPTY
    if b is None or b.is_empty:
        return a
    try:
        return a.difference(b)
    except Exception:
        return a.difference(b.buffer(0))


def count_overlaps(geoms) -> int:
    """统计互相重叠/相切的开孔对数（用于提示钢网强度）"""
    gs = [g for g in geoms if g is not None and not g.is_empty]
    if len(gs) < 2:
        return 0
    try:
        tree = shapely.STRtree(gs)
        hits = tree.query(gs, predicate="intersects")
        return int(sum(1 for i, j in zip(hits[0], hits[1]) if i < j))
    except Exception:
        return 0


def largest_poly(geom) -> Polygon | None:
    ps = polys_of(geom)
    if not ps:
        return None
    return max(ps, key=lambda p: p.area)


def quad_segs_for(radius: float, tol: float) -> int:
    """按弦高误差算圆的离散段数，返回**每象限**段数（整圆 = 4 * 返回值）

    shapely 的 buffer(quad_segs=n) 里 n 是每象限段数，别当成整圆段数用。
    """
    r = abs(radius)
    if r <= 0 or tol <= 0 or tol >= r:
        return 2
    n_full = math.pi / math.acos(max(-1.0, min(1.0, 1.0 - tol / r)))
    return int(max(1, min(24, math.ceil(n_full / 4.0))))


def circle(cx: float, cy: float, r: float, tol: float) -> Polygon:
    if r <= 0:
        return EMPTY
    return Point(cx, cy).buffer(r, quad_segs=quad_segs_for(r, tol))


def ring_to_poly(coords, ccw: bool) -> Polygon:
    pts = list(coords)
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        return EMPTY
    p = Polygon(pts)
    if not p.is_valid:
        p = p.buffer(0)
    return p


def signed_area(coords) -> float:
    a = 0.0
    n = len(coords)
    for i in range(n):
        x1, y1 = coords[i][0], coords[i][1]
        x2, y2 = coords[(i + 1) % n][0], coords[(i + 1) % n][1]
        a += x1 * y2 - x2 * y1
    return a * 0.5


def min_dimension(poly: Polygon) -> float:
    """开孔的最小可印宽度 = 最大内切圆直径。

    不能只用最小外接矩形：焊盘中间被挖掉一块之后是个环，外接矩形还是整个
    焊盘那么大（1.8x1.2），可壁只剩 0.05mm——外接矩形永远看不出这种薄壁，
    结果就是一条印不出来的细环被当成合格开孔，还把里面那块料围成孤岛。
    最大内切圆对环形、C 形都能正确给出壁厚。

    注意凹角：C 形开口内侧那个角上能塞进比臂宽更大的圆（实测 1.17 vs 臂宽
    1.0），"最细处"和"臂宽"本来就不是一回事。这里要的是最细处——它决定能
    不能印出来。
    """
    if poly is None or poly.is_empty or poly.area <= 0:
        return 0.0
    # shapely 2.1+ 有原生实现，返回圆心到最近边界的线段，长度就是内切圆半径。
    # 比下面二分快一个数量级（实测 123 个图元 0.25s vs 3.0s），结果一致。
    mic = getattr(shapely, "maximum_inscribed_circle", None)
    if mic is not None and poly.geom_type == "Polygon":
        try:
            return 2.0 * mic(poly).length
        except Exception:
            pass                                  # 退到二分
    try:
        # 内切圆半径不可能超过 sqrt(面积/π)，拿它当上界
        lo, hi = 0.0, math.sqrt(poly.area / math.pi)
        if not poly.buffer(-hi).is_empty:
            return 2.0 * hi
        for _ in range(20):                      # 2^-20 的精度，够用了
            mid = (lo + hi) / 2.0
            if poly.buffer(-mid).is_empty:
                hi = mid
            else:
                lo = mid
        return 2.0 * lo
    except Exception:
        return float("nan")


def snap_geom(geom, grid: float = 1e-4):
    """把坐标吸附到网格上，避免浮点毛刺（1e-4 mm = 0.1 µm）"""
    if geom is None or geom.is_empty:
        return geom
    try:
        return shapely.set_precision(geom, grid)
    except Exception:
        return geom


# ============================================================================
# 3. Gerber RS-274X 解析器
# ============================================================================
# 说明：只实现钢网/板框/钻孔需要的子集，但覆盖了实际导出文件里会出现的构造：
#   FS/MO/LP/AD/AM(宏)/G01,G02,G03/G74,G75/G36,G37/D01,D02,D03/M02
#   TF,TA,TO 等属性 -> 忽略；SR 步进重复 -> 不支持并告警
# ----------------------------------------------------------------------------

_WORD_RE = re.compile(r"([A-Za-z])([+-]?[0-9]*)")
# 宏参数里的数字：1 / 1.5 / .5 / 1. / 1e-5 都要认
_NUM_RE = r"(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?"
_ADD_RE = re.compile(r"^ADD(\d+)([A-Za-z_$.][A-Za-z0-9_$.\-]*)(?:,(.*))?$", re.S)
_AM_RE = re.compile(r"^AM([A-Za-z0-9_$.\-]+)\*?(.*)$", re.S)


def _eval_expr(expr: str, variables: dict) -> float:
    """宏参数表达式求值：支持 + - x / ( ) 和 $n 变量（自己写，避免 eval 风险）"""
    s = expr.replace("$", " $")
    # 数字必须带上小数点，否则 "-0.0209" 会被切成 ["-", "0", "0209"]，
    # 小数部分整个丢掉、求值成 0——Protel/Altium 系的宏参数全是写死的小数，
    # 一丢就是整颗芯片的焊盘全空。科学计数法也得认（repr 会给 1e-05）。
    toks = re.findall(r"\$[A-Za-z_][A-Za-z0-9_]*|\$?" + _NUM_RE
                      + r"|[+\-xX/()]", s)
    if not toks:
        return 0.0
    for i, t in enumerate(toks):
        if t.startswith("$"):
            key = t[1:]
            try:
                toks[i] = repr(float(variables.get(key, variables.get(int(key) if key.isdigit() else -1, 0.0))))
            except Exception:
                toks[i] = "0.0"
    pos = [0]

    def peek():
        return toks[pos[0]] if pos[0] < len(toks) else None

    def eat():
        t = peek()
        pos[0] += 1
        return t

    def parse_expr():
        v = parse_term()
        while peek() in ("+", "-"):
            op = eat()
            r = parse_term()
            v = v + r if op == "+" else v - r
        return v

    def parse_term():
        v = parse_factor()
        while peek() in ("x", "X", "/"):
            op = eat()
            r = parse_factor()
            if op == "/":
                v = v / r if abs(r) > 1e-12 else 0.0
            else:
                v = v * r
        return v

    def parse_factor():
        t = peek()
        if t is None:
            return 0.0
        if t == "(":
            eat()
            v = parse_expr()
            if peek() == ")":
                eat()
            return v
        if t in ("+", "-"):
            eat()
            v = parse_factor()
            return v if t == "+" else -v
        eat()
        try:
            return float(t)
        except ValueError:
            return 0.0

    try:
        return float(parse_expr())
    except Exception:
        return 0.0


class Aperture:
    __slots__ = ("code", "kind", "mods", "macro_name", "mods_raw")

    def __init__(self, code, kind, mods, macro_name=None, mods_raw=None):
        self.code = code
        self.kind = kind              # C / R / O / P / MACRO
        self.mods = mods              # 已求值的数值（mm）
        self.macro_name = macro_name
        self.mods_raw = mods_raw or []  # 宏参数原始表达式（可含 $1）


class Macro:
    """孔径宏定义"""

    def __init__(self, name, lines):
        self.name = name
        self.lines = [ln.strip() for ln in lines if ln.strip()]
        self.prims = []
        for ln in self.lines:
            ln = ln.rstrip("*").strip()
            if not ln or ln.startswith("0"):   # 0 = 注释
                continue
            parts = ln.split(",")
            try:
                code = int(float(parts[0].strip()))
            except ValueError:
                continue
            self.prims.append((code, [p.strip() for p in parts[1:]]))

    def build(self, mods: dict, tol: float, rotation: float = 0.0):
        """按 ADD 传入的 $n 生成几何（已含 exposure 处理）"""
        on, off = [], []
        for code, args in self.prims:
            try:
                g = self._prim(code, args, mods, tol, rotation)
            except Exception:
                g = None
                log(f"  [警告] 宏 {self.name} 的图元 {code} 解析失败，已跳过")
            if g is None or g.is_empty:
                continue
            # exposure: 图元第 0 个参数 1=加料 0=挖空（circle/line/outline/polygon）
            raw0 = args[0] if args else "1"
            try:
                expo = _eval_expr(raw0, mods)
            except Exception:
                expo = 1.0
            (on if expo >= 0.5 else off).append(g)
        res = U(on)
        if off:
            res = D(res, U(off))
        return res

    def _prim(self, code, args, mods, tol, rotation):
        v = lambda i, d=0.0: (_eval_expr(args[i], mods) if i < len(args) else d)

        if code == 1:      # 圆: exposure, diameter, cx, cy, [rotation]
            return circle(v(2), v(3), v(1) / 2.0, tol)
        if code in (2, 20):  # 矢量线: exposure, width, x1,y1,x2,y2, rotation
            w = v(1)
            p1 = self._rot(v(2), v(3), rotation)
            p2 = self._rot(v(4), v(5), rotation)
            return LineString([p1, p2]).buffer(max(w, 1e-4) / 2.0,
                                               cap_style=1,
                                               quad_segs=quad_segs_for(w / 2, tol))
        if code == 21:     # 中心线: exposure, width, height, cx, cy, rotation
            w, h = v(1), v(2)
            return self._rect(v(3), v(4), w, h, rotation)
        if code == 22:     # 左下角线: exposure, width, height, x, y, rotation
            w, h = v(1), v(2)
            return self._rect(v(3) + w / 2.0, v(4) + h / 2.0, w, h, rotation)
        if code == 4:      # 轮廓: exposure, n, x1,y1, ... xn,yn, rotation
            n = int(round(v(1)))
            pts = []
            for i in range(n):
                pts.append(self._rot(v(2 + 2 * i), v(3 + 2 * i), rotation))
            if len(pts) < 3:
                return None
            p = Polygon(pts)
            return p.buffer(0) if not p.is_valid else p
        if code == 5:      # 正多边形: exposure, vertices, cx, cy, diameter, rotation
            n = int(round(v(1)))
            cx, cy, dia = v(2), v(3), v(4)
            rot = v(5, 0.0) + rotation
            if n < 3 or dia <= 0:
                return None
            pts = [(cx + dia / 2.0 * math.cos(math.radians(rot + 360.0 * i / n)),
                    cy + dia / 2.0 * math.sin(math.radians(rot + 360.0 * i / n)))
                   for i in range(n)]
            return Polygon(pts)
        if code == 7:      # 热焊盘: cx, cy, outer_dia, inner_dia, gap, rotation
            cx, cy, od, idia, gap = v(0), v(1), v(2), v(3), v(4)
            outer = circle(cx, cy, od / 2.0, tol)
            inner = circle(cx, cy, idia / 2.0, tol)
            res = D(outer, inner)
            gap = max(gap, 0.0)
            for ang in (0.0, 90.0):
                a = math.radians(ang + rotation)
                w = od + 1.0
                cut = Polygon([
                    (cx - w * math.cos(a) - gap / 2 * math.sin(a), cy - w * math.sin(a) + gap / 2 * math.cos(a)),
                    (cx + w * math.cos(a) - gap / 2 * math.sin(a), cy + w * math.sin(a) + gap / 2 * math.cos(a)),
                    (cx + w * math.cos(a) + gap / 2 * math.sin(a), cy + w * math.sin(a) - gap / 2 * math.cos(a)),
                    (cx - w * math.cos(a) + gap / 2 * math.sin(a), cy - w * math.sin(a) - gap / 2 * math.cos(a)),
                ])
                res = D(res, cut)
            return res
        if code == 6:      # 摩尔纹: 忽略（钢网上不会用）
            return None
        return None

    @staticmethod
    def _rot(x, y, deg):
        if not deg:
            return (x, y)
        a = math.radians(deg)
        return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))

    def _rect(self, cx, cy, w, h, rotation):
        pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
        pts = [self._rot(px, py, rotation) for px, py in pts]
        pts = [(px + cx, py + cy) for px, py in pts]
        return Polygon(pts)


class GerberLayer:
    """一层 Gerber 的解析结果"""

    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.unit_scale = 1.0        # -> mm
        self.unit_name = "mm"
        self.int_digits = 3
        self.dec_digits = 4
        self.notation = "L"          # L=省略前导零  T=省略后导零
        self.apertures: dict[int, Aperture] = {}
        self.macros: dict[str, Macro] = {}
        self.items = []              # [(polarity, geometry, kind)]
        self.paths = []              # 中心线（用于板框重构）
        self.warnings = []
        self.file_function = ""      # %TF.FileFunction%

    # ---- 坐标 ----
    def _coord(self, raw: str) -> float:
        if raw in ("", "+", "-"):
            return 0.0
        sign = -1.0 if raw.startswith("-") else 1.0
        digits = raw.lstrip("+-")
        if not digits:
            return 0.0
        val = float(int(digits))
        if self.notation == "T":
            # 后导零省略：数字左对齐
            shift = self.int_digits - len(digits)
            val = val * (10.0 ** shift)
        else:
            # 前导零省略：数字右对齐
            val = val / (10.0 ** self.dec_digits)
        return sign * val * self.unit_scale

    # ---- 孔径 -> 形状 ----
    def aperture_shape(self, ap: Aperture, tol: float):
        m = ap.mods
        if ap.kind == "C":
            r = m[0] / 2.0
            g = circle(0, 0, r, tol)
            if len(m) > 1 and m[1] > 0:
                g = D(g, circle(0, 0, m[1] / 2.0, tol))
            return g
        if ap.kind == "R":
            w, h = m[0], m[1]
            g = box(-w / 2, -h / 2, w / 2, h / 2)
            if len(m) > 2 and m[2] > 0:
                g = D(g, circle(0, 0, m[2] / 2.0, tol))
            return g
        if ap.kind == "O":
            w, h = m[0], m[1]
            if w >= h:
                seg = LineString([(-(w - h) / 2.0, 0), ((w - h) / 2.0, 0)])
                g = seg.buffer(h / 2.0, cap_style=1, quad_segs=quad_segs_for(h / 2, tol))
            else:
                seg = LineString([(0, -(h - w) / 2.0), (0, (h - w) / 2.0)])
                g = seg.buffer(w / 2.0, cap_style=1, quad_segs=quad_segs_for(w / 2, tol))
            if len(m) > 2 and m[2] > 0:
                g = D(g, circle(0, 0, m[2] / 2.0, tol))
            return g
        if ap.kind == "P":
            dia, n = m[0], int(round(m[1]))
            rot = m[2] if len(m) > 2 else 0.0
            if n < 3:
                return EMPTY
            pts = [(dia / 2.0 * math.cos(math.radians(rot + 360.0 * i / n)),
                    dia / 2.0 * math.sin(math.radians(rot + 360.0 * i / n)))
                   for i in range(n)]
            g = Polygon(pts)
            if len(m) > 3 and m[3] > 0:
                g = D(g, circle(0, 0, m[3] / 2.0, tol))
            return g
        if ap.kind == "MACRO":
            mac = self.macros.get(ap.macro_name)
            if mac is None:
                return EMPTY
            # 宏体和 ADD 实参都在**文件单位**里，所以先按文件单位求值，
            # 建完形状再整体换算成 mm。混着来的话，$n 出来的值是 mm、
            # 宏里写死的小数还是英寸，同一个焊盘里两个尺度（Protel/Altium
            # 系的宏参数全是写死的小数，实测整颗芯片小成 1/25.4）。
            vals = {str(i + 1): self._macro_arg_raw(raw)
                    for i, raw in enumerate(ap.mods_raw)}
            # 容差也要按文件单位给：后面整体缩放会把误差一起放大，
            # 英寸文件里 0.01mm 的容差喂进去，缩放完就变成 0.25mm 的棱角
            g = mac.build(vals, tol / self.unit_scale)
            if abs(self.unit_scale - 1.0) > 1e-12 and g is not None \
                    and not g.is_empty:
                g = shapely.affinity.scale(g, xfact=self.unit_scale,
                                           yfact=self.unit_scale)
            return g
        return EMPTY

    def _macro_arg_raw(self, raw: str) -> float:
        """ADD 里的宏实参，按**文件单位**求值（不换算）"""
        raw = raw.strip()
        if not raw:
            return 0.0
        try:
            return float(raw)
        except ValueError:
            pass
        # $2x3 之类：按宏内表达式处理（这里的变量表来自同一 ADD 的前序参数）
        return _eval_expr(raw, getattr(self, "_cur_macro_vars", {}))

    def _eval_macro_arg(self, raw: str) -> float:
        """同上，但换算到 mm（C/R/O/P 这些标准光圈用得到）"""
        return self._macro_arg_raw(raw) * self.unit_scale

    # ---- 主解析 ----
    def parse(self, tol: float) -> "GerberLayer":
        with open(self.path, "r", errors="replace") as f:
            text = f.read()
        text = text.replace("\r", "")

        # 状态
        cx = cy = 0.0
        cur_ap: Aperture | None = None
        interp = "linear"          # linear / cw / ccw
        quadrant = "multi"         # multi / single
        polarity = "dark"
        in_region = False
        region_pts: list[tuple[float, float]] = []
        region_cw = False
        image_negative = False
        got_fs = False
        sr_warned = False
        modal_op = None            # D01/D02/D03 是模态的，见下面 op 的取法
        flash_lost: set[int] = set()   # 已经报过"这个光圈建不出来"的光圈号

        def flush_region():
            nonlocal region_pts
            if len(region_pts) >= 3:
                p = Polygon(region_pts)
                if not p.is_valid:
                    p = p.buffer(0)
                if not p.is_empty and p.area > 0:
                    self.items.append((polarity, p, "region"))
            region_pts = []

        def add_line(p1, p2, ap: Aperture | None):
            """D01 画线。光圈未定义或零宽时只记中心线——板框经常这么画，
            而板框恰恰是靠中心线 polygonize 出来的，丢了就整块板都没了。"""
            if p1 == p2:
                return
            self.paths.append(LineString([p1, p2]))
            if ap is None:
                return
            if ap.kind == "C":
                r = ap.mods[0] / 2.0
                if r <= 1e-9:
                    return                                   # 零宽光圈
                g = LineString([p1, p2]).buffer(r, cap_style=1,
                                                quad_segs=quad_segs_for(r, tol))
            else:
                # 非圆孔径画线：近似处理
                shape = self.aperture_shape(ap, tol)
                half = min(ap.mods[0], ap.mods[1]) / 2.0 if len(ap.mods) >= 2 else 0.05
                g = LineString([p1, p2]).buffer(max(half, 1e-4), cap_style=2, join_style=2)
                g = U([g, self._place(shape, p1), self._place(shape, p2)])
            self.items.append((polarity, g, "draw"))

        def arc_points(p0, p1, i, j, cw: bool):
            """把圆弧离散成折线点（含起点终点）"""
            x0, y0 = p0
            x1, y1 = p1
            if quadrant == "single":
                k = -1.0 if cw else 1.0
                ccx, ccy = x0 + k * abs(i), y0 + k * abs(j)
            else:
                ccx, ccy = x0 + i, y0 + j
            r = math.hypot(x0 - ccx, y0 - ccy)
            if r < 1e-9:
                return [p0, p1]
            a0 = math.atan2(y0 - ccy, x0 - ccx)
            a1 = math.atan2(y1 - ccy, x1 - ccx)
            if cw:
                while a1 >= a0 - 1e-12:
                    a1 -= 2 * math.pi
            else:
                while a1 <= a0 + 1e-12:
                    a1 += 2 * math.pi
            sweep = abs(a1 - a0)
            n = max(2, int(math.ceil(sweep / (2 * math.acos(
                max(-1.0, min(1.0, 1.0 - min(tol, r * 0.9) / r)))))))
            n = min(n, 2000)
            pts = [(ccx + r * math.cos(a0 + (a1 - a0) * t / n),
                    ccy + r * math.sin(a0 + (a1 - a0) * t / n)) for t in range(n + 1)]
            # 首尾必须用精确端点：极坐标反算有 ~1e-16 误差，会让 polygonize 接不上
            pts[0] = (x0, y0)
            pts[-1] = (x1, y1)
            return pts

        # ---- 扫描指令 ----
        i = 0
        n = len(text)
        while i < n:
            ch = text[i]
            if ch in " \t\n":
                i += 1
                continue
            if ch == "%":
                end = text.find("%", i + 1)
                if end < 0:
                    end = n
                body = text[i + 1:end]
                i = end + 1
                # 孔径宏必须整块吃，不能按 * 拆开：%AMOval*图元*图元*...%
                # 里每个图元自己就是一个 * 段，拆开之后宏就只剩个名字，
                # 图元全丢——建出来是空形状，用它的闪光静默消失。
                # 立创EDA 的 IC 焊盘全是圆角矩形宏，整颗芯片就这么没了。
                if body.lstrip().upper().startswith("AM"):
                    mm = _AM_RE.match(body.lstrip())
                    if mm:
                        self.macros[mm.group(1)] = Macro(
                            mm.group(1), mm.group(2).split("*"))
                    continue
                for cmd in body.split("*"):
                    cmd = cmd.strip()
                    if not cmd:
                        continue
                    up = cmd.upper()
                    if up.startswith("FS"):
                        # 零省略方式：L=省略前导零 T=省略后导零 D=不省略。
                        # D 是老规范的写法（现规范已废弃），立创EDA 至今照导，
                        # 只认 [LT] 就会匹配失败、悄悄退到默认 3.4，坐标整体差出
                        # 一个数量级。D 是定宽的，数值处理和 L 完全一样。
                        mm = re.match(r"FS([LTD])([AI])X(\d)(\d)Y(\d)(\d)", up)
                        if mm:
                            self.notation = "T" if mm.group(1) == "T" else "L"
                            self.int_digits = int(mm.group(3))
                            self.dec_digits = int(mm.group(4))
                            got_fs = True
                            if mm.group(3) != mm.group(5) or mm.group(4) != mm.group(6):
                                warn(f"{self.name}: X/Y 坐标位数不同"
                                     f"（X{mm.group(3)}.{mm.group(4)} "
                                     f"Y{mm.group(5)}.{mm.group(6)}），按 X 的位数解析")
                    elif up.startswith("MO"):
                        if "IN" in up:
                            self.unit_scale = 25.4
                            self.unit_name = "inch"
                        else:
                            self.unit_scale = 1.0
                            self.unit_name = "mm"
                    elif up.startswith("LP"):
                        polarity = "clear" if "C" in up[2:3] else "dark"
                    elif up.startswith("IP"):
                        if "NEG" in up:
                            image_negative = True
                    elif up.startswith("ADD"):
                        mm = _ADD_RE.match(cmd)
                        if mm:
                            code = int(mm.group(1))
                            kind = mm.group(2).upper()
                            raws = [x.strip() for x in (mm.group(3) or "").split("X")]
                            vals = []
                            self._cur_macro_vars = {}
                            for k, r in enumerate(raws):
                                fv = self._eval_macro_arg(r)
                                self._cur_macro_vars[str(k + 1)] = fv / (self.unit_scale or 1.0)
                                vals.append(fv)
                            if kind in ("C", "R", "O", "P"):
                                self.apertures[code] = Aperture(code, kind, vals)
                            else:
                                self.apertures[code] = Aperture(
                                    code, "MACRO", vals, macro_name=mm.group(2),
                                    mods_raw=raws)
                    elif up.startswith("SR"):
                        # SRX1Y1 就是"不重复"，立创每个文件都写，属正常情况不该报警
                        rep = re.match(r"SRX(\d+)Y(\d+)", up)
                        if not (rep and rep.group(1) == "1" and rep.group(2) == "1") \
                                and not sr_warned:
                            warn(f"{self.name}: 含 SR 步进重复指令，暂不支持，按单图形处理")
                            sr_warned = True
                    elif up.startswith("TF"):
                        mm = re.match(r"TF\.FileFunction,(.*)", cmd, re.I)
                        if mm:
                            self.file_function = mm.group(1).strip()
                    # 其它扩展参数（TA/TO/TD/IR/MI/OF/SF/AS/IN/LN...）忽略
                continue

            # ---- 普通指令（以 * 结束）----
            end = text.find("*", i)
            if end < 0:
                end = n
            word = text[i:end].strip()
            i = end + 1
            if not word:
                continue
            up = word.upper()

            if up.startswith("G04") or up.startswith("G4"):
                continue
            if up.startswith("M02") or up.startswith("M00") or up.startswith("M01"):
                break

            # 提取 字母+数字
            pairs = _WORD_RE.findall(up)
            g_codes = [int(v) for k, v in pairs if k == "G" and v not in ("",)]
            has_xy = any(k in ("X", "Y") for k, _ in pairs)
            d_code = None
            for k, v in pairs:
                if k == "D" and v:
                    d_code = int(v)

            for g in g_codes:
                if g == 1:
                    interp = "linear"
                elif g == 2:
                    interp = "cw"
                elif g == 3:
                    interp = "ccw"
                elif g == 74:
                    quadrant = "single"
                elif g == 75:
                    quadrant = "multi"
                elif g == 36:
                    in_region = True
                    region_pts = []
                elif g == 37:
                    if in_region:
                        flush_region()
                    in_region = False
                elif g == 70:
                    self.unit_scale, self.unit_name = 25.4, "inch"
                elif g == 71:
                    self.unit_scale, self.unit_name = 1.0, "mm"
                elif g == 90:
                    pass
                elif g == 91:
                    warn(f"{self.name}: 不支持增量坐标(G91)，已忽略")

            # 孔径选择
            if d_code is not None and d_code >= 10 and not has_xy:
                cur_ap = self.apertures.get(d_code)
                continue

            if not has_xy and d_code is None:
                continue

            nx, ny = cx, cy
            ii = jj = 0.0
            for k, v in pairs:
                if k == "X":
                    nx = self._coord(v)
                elif k == "Y":
                    ny = self._coord(v)
                elif k == "I":
                    ii = self._coord(v)
                elif k == "J":
                    jj = self._coord(v)

            # D01/D02/D03 是模态的：坐标块里不写 D 码就沿用上一次的操作。
            # 立创EDA 的板框真的这么省——"X0324000*" 光有坐标，意思就是画一条边。
            # 只认字面出现的 D 码，这条边会被整条丢掉，板框就闭合不了。
            if d_code in (1, 2, 3):
                modal_op = d_code
            op = modal_op
            p0 = (cx, cy)
            p1 = (nx, ny)

            if op == 1:      # 画
                if interp == "linear":
                    if in_region:
                        if not region_pts:
                            region_pts.append(p0)
                        region_pts.append(p1)
                    else:
                        add_line(p0, p1, cur_ap)
                else:
                    pts = arc_points(p0, p1, ii, jj, interp == "cw")
                    if in_region:
                        if not region_pts:
                            region_pts.append(pts[0])
                        region_pts.extend(pts[1:])
                    else:
                        for a, b in zip(pts, pts[1:]):
                            add_line(a, b, cur_ap)
            elif op == 2:    # 移动
                if in_region and region_pts:
                    flush_region()
                if in_region:
                    region_pts = [p1]
            elif op == 3:    # 闪光
                if cur_ap is None:
                    warn(f"{self.name}: 闪光时未选择孔径 (D{d_code})，已跳过")
                else:
                    g = self.aperture_shape(cur_ap, tol)
                    if g is not None and not g.is_empty:
                        self.items.append((polarity, self._place(g, p1), "flash"))
                    elif cur_ap.code not in flash_lost:
                        # 光圈建不出形状，用它的闪光就全没了。以前这里是静默
                        # 丢弃，整颗芯片少掉都不出声——出过一次事，必须报出来。
                        flash_lost.add(cur_ap.code)
                        what = (f"宏 {cur_ap.macro_name}" if cur_ap.kind == "MACRO"
                                else f"{cur_ap.kind} 形光圈")
                        warn(f"{self.name}: 光圈 D{cur_ap.code}（{what}）"
                             f"建不出形状，用它的锡膏开孔被整批跳过")

            cx, cy = nx, ny

        if not got_fs:
            warn(f"{self.name}: 未找到 %FS% 坐标格式，按默认 3.4 前导零省略解析")
        if image_negative:
            warn(f"{self.name}: 图像负片(IPNEG)，结果可能不符预期")
        return self

    @staticmethod
    def _place(geom, pos):
        if pos == (0.0, 0.0):
            return geom
        return shapely.affinity.translate(geom, pos[0], pos[1])


# shapely.affinity 需要显式导入
import shapely.affinity  # noqa: E402


# ============================================================================
# 4. Excellon 钻孔解析器
# ============================================================================
class ExcellonLayer:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.unit_scale = 1.0
        self.unit_name = "mm"
        self.file_format = None        # (int_digits, dec_digits)
        self.zero_mode = None          # 'TZ' / 'LZ'
        self.tools: dict[int, float] = {}
        self.hits: list[tuple[float, float, float]] = []   # (x, y, dia) mm
        self.slots: list[tuple[tuple[float, float], tuple[float, float], float]] = []
        self.warnings = []

    def _num(self, raw: str, default_dec: int) -> float:
        raw = raw.strip()
        if raw in ("", "+", "-"):
            return 0.0
        if "." in raw:
            return float(raw) * self.unit_scale
        sign = -1.0 if raw.startswith("-") else 1.0
        digits = raw.lstrip("+-")
        if not digits:
            return 0.0
        v = float(int(digits))
        if self.zero_mode == "LZ" and self.file_format:
            a, b = self.file_format
            v = v * (10.0 ** (a - len(digits)))
        else:
            v = v / (10.0 ** default_dec)
        return sign * v * self.unit_scale

    def parse(self, tol: float) -> "ExcellonLayer":
        with open(self.path, "r", errors="replace") as f:
            lines = [ln.strip() for ln in f.read().replace("\r", "").split("\n")]

        in_header = False
        header_done = False
        cur_tool = None
        body_coords: list[str] = []
        # 先扫一遍收集所有坐标串，用于推断小数位
        for ln in lines:
            s = ln.split(";")[0].strip()
            if not s:
                continue
            if re.match(r"^[XY][+-]?\d", s):
                body_coords.append(s)

        dec_guess = self._guess_decimals(body_coords)

        for ln in lines:
            raw = ln
            s = raw.split(";")[0].strip().upper()
            if not s:
                continue
            if "FILE_FORMAT" in raw.upper():
                mm = re.search(r"FILE_FORMAT\s*=\s*(\d+)\s*:\s*(\d+)", raw, re.I)
                if mm:
                    self.file_format = (int(mm.group(1)), int(mm.group(2)))
                continue
            if s.startswith("M48"):
                in_header = True
                continue
            if s.startswith("M95") or s == "%":
                in_header = False
                header_done = True
                continue
            if s.startswith("M30") or s.startswith("M00") or s.startswith("M01"):
                break
            if s.startswith("METRIC"):
                self.unit_scale, self.unit_name = 1.0, "mm"
                if "LZ" in s:
                    self.zero_mode = "LZ"
                elif "TZ" in s:
                    self.zero_mode = "TZ"
                continue
            if s.startswith("INCH"):
                self.unit_scale, self.unit_name = 25.4, "inch"
                if "LZ" in s:
                    self.zero_mode = "LZ"
                elif "TZ" in s:
                    self.zero_mode = "TZ"
                continue
            if s.startswith("M71"):
                self.unit_scale, self.unit_name = 1.0, "mm"
                continue
            if s.startswith("M72"):
                self.unit_scale, self.unit_name = 25.4, "inch"
                continue
            if s.startswith("FMAT") or s.startswith("VER") or s.startswith("G90") \
                    or s.startswith("G05") or s.startswith("G81") or s.startswith("M47") \
                    or s.startswith("ICI") or s.startswith("DETECT") or s.startswith("ATC"):
                continue

            # 刀具定义 T1C0.800
            mm = re.match(r"^T(\d+)C([0-9.]+)", s)
            if mm:
                self.tools[int(mm.group(1))] = float(mm.group(2)) * self.unit_scale
                continue
            if re.match(r"^T(\d+)F[SC]", s):     # T1F S / T1F C 进给
                continue
            mm = re.match(r"^T(\d+)\s*$", s)
            if mm:
                cur_tool = int(mm.group(1))
                if cur_tool not in self.tools:
                    self.tools[cur_tool] = 0.0
                continue
            if s.startswith("R") and re.match(r"^R\d+", s):
                continue

            # 槽孔 X..Y..G85X..Y..
            mm = re.match(r"^X([+-]?[\d.]+)Y([+-]?[\d.]+)G85X([+-]?[\d.]+)Y([+-]?[\d.]+)", s)
            if mm:
                p1 = (self._num(mm.group(1), dec_guess), self._num(mm.group(2), dec_guess))
                p2 = (self._num(mm.group(3), dec_guess), self._num(mm.group(4), dec_guess))
                dia = self.tools.get(cur_tool, 0.0)
                if dia > 0:
                    self.slots.append((p1, p2, dia))
                continue

            mm = re.match(r"^X([+-]?[\d.]*)(?:Y([+-]?[\d.]*))?", s)
            if mm and (mm.group(1) or mm.group(2)):
                x = self._num(mm.group(1) or "0", dec_guess)
                y = self._num(mm.group(2) or "0", dec_guess)
                dia = self.tools.get(cur_tool, 0.0)
                if dia <= 0:
                    self.warnings.append(f"{self.name}: 刀具 T{cur_tool} 直径未知，该孔已跳过")
                    continue
                self.hits.append((x, y, dia))
                continue

        if not self.tools:
            self.warnings.append(f"{self.name}: 未解析到任何刀具定义")
        if not self.hits and not self.slots:
            self.warnings.append(f"{self.name}: 未解析到任何钻孔")
        return self

    def _guess_decimals(self, coords: list[str]) -> int:
        """没有小数点时推断小数位数：选一个让板子尺寸最合理的结果"""
        nums = []
        for s in coords:
            for m in re.finditer(r"[XY]([+-]?\d+)", s):
                nums.append(m.group(1).lstrip("+-"))
        if not nums:
            return 3 if self.unit_scale == 1.0 else 4
        if any("." in s for s in coords):
            return 3
        maxlen = max(len(x) for x in nums)
        cands = [2, 3, 4] if self.unit_scale == 1.0 else [3, 4, 5]
        best, best_score = cands[0], -1e9
        for b in cands:
            vals = [int(x) / (10.0 ** b) * self.unit_scale for x in nums]
            span = max(vals) - min(vals)
            score = 0.0
            if 1.0 <= span <= 700:
                score += 10.0
            score -= abs(span - 80.0) / 100.0
            if span > 2000:
                score -= 50
            if len(nums) and maxlen - b < 1:
                score -= 20
            if score > best_score:
                best, best_score = b, score
        return best

    def geometry(self, tol: float, min_dia: float = 0.0):
        """返回钻孔多边形列表。只返回直径 > min_dia 的孔——
        比它小的当做过孔（盘中孔），要留给焊盘，不能拿来做避让。"""
        geoms = []
        for x, y, d in self.hits:
            if d > min_dia:
                geoms.append(circle(x, y, d / 2.0, tol))
        for p1, p2, d in self.slots:
            if d <= min_dia:
                continue
            r = d / 2.0
            if math.dist(p1, p2) < 1e-9:
                geoms.append(circle(p1[0], p1[1], r, tol))
            else:
                geoms.append(LineString([p1, p2]).buffer(
                    r, cap_style=1, quad_segs=quad_segs_for(r, tol)))
        return geoms

    def dia_counts(self, via_dia: float):
        """返回 (过孔数, 真孔数)：直径 ≤ via_dia 的算过孔，不参与避让"""
        n_via = (sum(1 for _, _, d in self.hits if d <= via_dia)
                 + sum(1 for _, _, d in self.slots if d <= via_dia))
        return n_via, len(self.hits) + len(self.slots) - n_via


# ============================================================================
# 5. 文件识别
# ============================================================================
def _read_head(path: str, n: int = 8192) -> str:
    try:
        with open(path, "rb") as f:
            return f.read(n).decode("ascii", "ignore")
    except OSError:
        return ""


def _content_kind(head: str) -> str:
    """按文件内容判断格式：'gerber' / 'excellon' / ''（认不出来）"""
    if not head:
        return ""
    gerber = re.search(r"%FS|%MO|%AD|%AM|%LP|%IP|G36\*|G37\*", head, re.I)
    excellon = re.search(r"(?:^|\n)\s*M48\b|(?:^|\n)\s*T\d+C\s*[\d.]|"
                         r"(?:^|\n)\s*(?:METRIC|INCH)\s*[,T]", head, re.I)
    if excellon and not gerber:
        return "excellon"
    if gerber and not excellon:
        return "gerber"
    if gerber:                                   # 两者都像时 Gerber 的特征更硬
        return "gerber"
    if excellon:
        return "excellon"
    return ""


# 正反面判据。按"词"匹配而不是子串匹配：KiCad 把底层钢网叫 B_Paste.gbr，
# 分隔符在 b 后面，老代码只找 "_b_"/"-b_" 之类，一个都对不上，
# 结果底层被判成顶层——而且 B_ 按字母序还排在 F_ 前面，取第一个就真拿错了层。
_SIDE_BOT = {"b", "bot", "bottom", "back", "l2", "l4",
             "gbp", "gbo", "gbs", "gbl", "gb"}
_SIDE_TOP = {"f", "top", "front", "l1", "l3",
             "gtp", "gto", "gts", "gtl", "gt"}


def _layer_side(name: str) -> str | None:
    """从文件名判断是正面还是反面，判不出来返回 None"""
    low = name.lower()
    for tok in re.split(r"[^a-z0-9一-鿿]+", low):
        if tok in _SIDE_BOT:
            return "bottom"
        if tok in _SIDE_TOP:
            return "top"
    if "bottom" in low or "背面" in name or "底层" in name or "底" in name:
        return "bottom"
    if "top" in low or "正面" in name or "顶层" in name or "顶" in name:
        return "top"
    return None


def classify_by_attribute(path: str, head: str | None = None) -> str | None:
    """优先用文件里的 X2 属性判层（比文件名可靠，AD/KiCad/立创新版本都会写）。

    Gerber  : %TF.FileFunction,Paste,Top*%
    Excellon: ; #@! TF.FileFunction,Plated,1,2,PTH
    """
    if head is None:
        head = _read_head(path)
    if not head:
        return None
    m = re.search(r"%TF\.FileFunction,([^*%]+)[*%]", head, re.I)
    if not m:
        m = re.search(r"#@!\s*TF\.FileFunction,([^\r\n*]+)", head, re.I)
    if not m:
        return None
    fn = m.group(1).strip().lower()
    if "paste" in fn:
        side = "bottom" if re.search(r"\b(bot|bottom|l2|l4|b)\b", fn) else "top"
        return "paste_" + side
    if "profile" in fn or "outline" in fn or "board" in fn:
        return "outline"
    if "drill" in fn or "pth" in fn:                    # NonPlated 里也含 plated，先判非金属化
        if "npth" in fn or "nonplated" in fn or "non_plated" in fn or "non-plated" in fn:
            return "drill_npth"
        if "pth" in fn or "plated" in fn:
            return "drill_pth"
        return "drill"
    if "soldermask" in fn or "mask" in fn:
        return "mask"
    if "legend" in fn or "silk" in fn:
        return "silk"
    if "copper" in fn:
        return "copper"
    if "assembly" in fn or "glue" in fn or "carbon" in fn or "other" in fn:
        return "other"
    return None


def classify_file(path: str) -> str:
    """按文件内 X2 属性 + 文件名判断层类型：paste_top / paste_bottom / outline /
    drill_pth / drill_npth / drill / copper / mask / silk / other"""
    head = _read_head(path)
    k = classify_by_attribute(path, head)
    if k:
        return k
    n = os.path.basename(path).lower()
    stem, ext = os.path.splitext(n)
    kind = _content_kind(head)
    # 再去掉一切分隔符留一份。Altium 不勾 "Use Protel filename extensions" 导出的是
    # "MyBoard-Keep-Out Layer.gbr"，层名里的空格连字符都在，直接找 "keepout" 对不上。
    flat = re.sub(r"[^a-z0-9一-鿿]+", "", n)

    def has(*keys):
        return any(k in n or k in flat for k in keys)

    # 钻孔：名字像钻孔还不够，内容也得是 Excellon。
    # Gerber 格式的"钻孔图"(DrillDrawing/GDD) 只是示意图，不是钻孔数据；
    # 说明文件(README.txt) 更不是。
    looks_drill = ext in (".drl", ".ncd", ".xln", ".drr") or has("drill", "nc_", "钻孔")
    if ext == ".txt" or has("readme", "说明", "必读", "下单"):
        looks_drill = kind == "excellon"
    if looks_drill:
        if kind == "gerber":
            return "other"                       # 钻孔示意图，忽略
        if has("npth", "non_plated", "nonplated", "npt"):
            return "drill_npth"
        if has("pth", "plated"):
            return "drill_pth"
        return "drill"
    # 板框。Keep-Out 就是 AD 里画板子形状的那层，中文版叫"禁止布线层"
    if has("outline", "edge_cuts", "edgecuts", "boardoutline", "board_outline",
           "gko", "keepout", "border", "板框", "禁止布线"):
        return "outline"
    # 机械层：AD 官方也建议拿它画板框，但机械层上同样可能画的是尺寸标注、
    # 装配图，所以单独归一类，只有找不到正经板框时才拿来顶。
    # 只认"机械层"不认"机械"，免得"机械孔"这种被当成板框。
    if has("gm1", "gml", "mechanical", "机械层"):
        return "outline_weak"
    # 钢网（锡膏层）
    if has("paste", "gtp", "gbp", "锡膏", "钢网"):
        return "paste_bottom" if _layer_side(n) == "bottom" else "paste_top"
    if ext == ".gtp":
        return "paste_top"
    if ext == ".gbp":
        return "paste_bottom"
    # 其它层
    if has("mask", "gts", "gbs", "阻焊"):
        return "mask"
    if has("silk", "gto", "gbo", "丝印"):
        return "silk"
    if ext in (".gtl", ".gbl", ".g1", ".g2", ".gbr", ".ger", ".art", ".gdo", ".gds") or has("copper"):
        return "copper"
    return "other"


def scan_input(path: str, log_prefix: str = "") -> dict[str, list[str]]:
    """扫描目录或 zip，返回 类别 -> 文件列表"""
    if os.path.isdir(path):
        root = path
    elif os.path.isfile(path) and path.lower().endswith(".zip"):
        root = tempfile.mkdtemp(prefix="pcb_gerber_")
        with zipfile.ZipFile(path) as z:
            z.extractall(root)
        log(f"{log_prefix}已解压 zip -> {root}")
    elif os.path.isfile(path):
        return {classify_file(path): [path]}
    else:
        raise JobError(f"输入路径不存在: {path}")

    found: dict[str, list[str]] = {}
    for dirpath, _, files in os.walk(root):
        for fn in files:
            fp = os.path.join(dirpath, fn)
            if os.path.getsize(fp) == 0:
                continue
            k = classify_file(fp)
            found.setdefault(k, []).append(fp)
    return found


# ============================================================================
# 6. 主流程：PCB -> 钢网 + 治具
# ============================================================================
@dataclass
class Band:
    """分层带：z0~z1 之间截面为 poly 的一段实体

    top_face / bottom_face:
        None  = 用 poly 本身（整面露出）
        EMPTY = 该面被相邻带完全盖住，不输出面
        Polygon = 只露出这个区域（接口面）
    接口面必须直接复用已有的多边形（而不是重新做布尔差集），
    否则顶点坐标会出现极小的偏差，网格就不闭合了。
    """
    poly: Polygon
    z0: float
    z1: float
    name: str = ""
    top_face: Polygon | None = None
    bottom_face: Polygon | None = None


@dataclass
class JobResult:
    bands: list[Band] = field(default_factory=list)
    stats: dict = field(default_factory=dict)
    board: Polygon | None = None
    plate: Polygon | None = None
    pocket: Polygon | None = None
    jig_out: Polygon | None = None
    apertures: object = None
    dropped: object = None
    holes: object = None
    mesh: list = field(default_factory=list)


def pick_files(found: dict[str, list[str]], layer: str) \
        -> tuple[str, str | None, list[str], str]:
    """返回 (钢网文件, 板框文件, 钻孔文件列表, 实际用到的层)。

    要的层不存在时会退回另一层，但会明确告警——静默回退会让人以为程序没反应。
    """
    cn = {"top": "顶层", "bottom": "底层"}

    def _pick(kind: str) -> str | None:
        """同一类里挑一个。多候选时必须说清楚挑了哪个：
        导出目录里同时有顶层和底层钢网时，不吭声地取第一个等于让用户猜。"""
        lst = sorted(found.get(f"paste_{kind}", []))       # 排序，保证结果可复现
        if not lst:
            return None
        if len(lst) > 1:
            warn(f"！！ 找到 {len(lst)} 个{cn[kind]}锡膏层文件，将使用第一个，"
                 f"其余的忽略：")
            for i, fp in enumerate(lst):
                warn(f"！！   {'→' if i == 0 else ' '} {os.path.basename(fp)}")
        return lst[0]

    p_top, p_bot = _pick("top"), _pick("bottom")
    used = layer
    if layer == "top":
        paste = p_top or p_bot
        if p_top is None and p_bot is not None:
            used = "bottom"
    else:
        paste = p_bot or p_top
        if p_bot is None and p_top is not None:
            used = "top"

    if paste is None:
        raise JobError("没有找到钢网层（锡膏层）。请确认导出了 Paste 层：\n"
                       "  Altium: *.GTP / *.GBP\n"
                       "  立创EDA: Gerber_TopPasteLayer.GTP / Gerber_BottomPasteLayer.GBP\n"
                       "  KiCad : *-F_Paste.gbr / *-B_Paste.gbr\n"
                       "  通用 : 文件名里带 paste/锡膏/钢网 等字样，或用 X2 格式导出\n"
                       "         （只叫 layer1.gbr 这种，文件里外都没层信息，认不出来）")
    if used != layer:
        warn(f"！！ 没有找到{cn[layer]}锡膏层，已改用{cn[used]}："
             f"{os.path.basename(paste)}")
        warn(f"！！ 如果这块板两面都有贴片，请检查导出时是否漏了 "
             f"{'Bottom/GBP' if layer == 'bottom' else 'Top/GTP'} 层")

    outlines = sorted(found.get("outline", []))
    if not outlines:
        # 只有机械层可用时凑合用，但得让用户知道用的是哪个、可能要担什么风险
        weak = sorted(found.get("outline_weak", []))
        if weak:
            warn(f"！！ 没有找到 Keep-Out / Edge_Cuts 这类明确的板框层，"
                 f"改用机械层：{os.path.basename(weak[0])}")
            warn(f"！！ 机械层上如果画的是尺寸标注、装配图之类，板子外形会是错的。"
                 f"建议改用 Keep-Out 层（中文版叫禁止布线层）重导一次")
            outlines = weak

    if not outlines:
        raise JobError("没有找到板框层。请确认导出了板框层：\n"
                       "  Altium: Keep-Out Layer（禁止布线层）/ Mechanical 1 / *.GKO\n"
                       "  立创EDA: Gerber_BoardOutlineLayer.GKO\n"
                       "  KiCad : *-Edge_Cuts.gbr\n"
                       "  通用 : 文件名里带 outline/edge_cuts/板框 等字样")
    if len(outlines) > 1:
        warn(f"！！ 找到 {len(outlines)} 个板框文件，将使用第一个：")
        for i, fp in enumerate(outlines):
            warn(f"！！   {'→' if i == 0 else ' '} {os.path.basename(fp)}")
    drills = (found.get("drill", []) + found.get("drill_pth", [])
              + found.get("drill_npth", []))
    return paste, outlines[0], drills, used


def build_outline(layer: GerberLayer, tol: float) -> Polygon:
    """从板框层得到板子外形"""
    # 1) 优先用中心线重构闭合轮廓
    if layer.paths:
        net = U(layer.paths)
        try:
            cands = [p for p in polygonize(net) if p.area > 0]
        except Exception:
            cands = []
        if cands:
            cands.sort(key=lambda p: p.area, reverse=True)
            board = cands[0]
            inner = [p for p in cands[1:] if board.contains(p.representative_point())]
            if inner:
                board = D(board, U(inner))
                log(f"  板框：识别到 {len(inner)} 个内部开槽")
            return board.buffer(0)
    # 2) 退化：用粗线宽外扩再回缩
    geoms = [g for _, g, _ in layer.items]
    merged = U(geoms)
    p = largest_poly(merged)
    if p is None:
        raise JobError("板框层解析结果为空，无法确定板框")
    widths = [ap.mods[0] for ap in layer.apertures.values() if ap.kind == "C" and ap.mods]
    w = max(widths) if widths else 0.1
    shrunk = p.buffer(-w / 2.0)
    if not shrunk.is_empty:
        lp = largest_poly(shrunk)
        if lp is not None and lp.area > 0:
            log(f"  板框：由线宽 {w:.3f}mm 的轮廓重构")
            return lp
    return p


def run_job(input_path: str, params: Params) -> JobResult:
    t_start = time.time()
    log("=" * 74)
    log(f"{APP_TITLE}  v{APP_VERSION}")
    log("=" * 74)

    found = scan_input(input_path)
    log(f"\n[1/6] 扫描输入：{os.path.abspath(input_path)}")
    for k in sorted(found):
        for fp in found[k]:
            log(f"      {k:<13} {os.path.basename(fp)}")

    paste_path, outline_path, drill_paths, layer_used = pick_files(found, params.layer)
    if layer_used != params.layer:
        warn(f"！！ 本次实际用的是{'顶层' if layer_used == 'top' else '底层'}锡膏数据")
    log(f"\n[2/6] 解析 Gerber")
    log(f"      钢网层 : {os.path.basename(paste_path)}"
        f"（{'顶层' if layer_used == 'top' else '底层'}）")
    log(f"      板框层 : {os.path.basename(outline_path)}")
    log(f"      钻孔层 : {', '.join(os.path.basename(p) for p in drill_paths) or '(无)'}")

    tol = params.arc_tolerance

    # ---- 钢网开孔 ----
    paste = GerberLayer(paste_path).parse(tol)
    for w in paste.warnings:
        warn(w)
    ap_instances = [(g, kind) for pol, g, kind in paste.items if kind in ("flash", "region")]
    if not ap_instances:
        raise JobError("钢网层里没有解析到任何开孔，请检查文件是否为锡膏层")
    flashes = [g for g, k in ap_instances if k == "flash"]
    regions = [g for g, k in ap_instances if k == "region"]
    log(f"      开孔图元：{len(flashes)} 个 flash + {len(regions)} 个区域填充，"
        f"单位 {paste.unit_name}")

    # ---- 板框 ----
    outline = GerberLayer(outline_path).parse(tol)
    for w in outline.warnings:
        warn(w)
    board = snap_geom(build_outline(outline, tol))
    if board is None or board.is_empty:
        raise JobError("板框解析失败")
    bx0, by0, bx1, by1 = board.bounds
    log(f"      板框尺寸：{bx1 - bx0:.2f} x {by1 - by0:.2f} mm，面积 {board.area:.1f} mm²")

    # ---- 钻孔（打穿孔禁布区）----
    hole_geoms = []
    n_hits = n_via = 0
    for dp in drill_paths:
        ex = ExcellonLayer(dp).parse(tol)
        gs = ex.geometry(tol, min_dia=params.via_dia)
        if not ex.hits and not ex.slots:
            log(f"      跳过 {os.path.basename(dp)}：里面没有钻孔数据")
            continue
        for w in ex.warnings:
            warn(w)
        n_via += ex.dia_counts(params.via_dia)[0]
        n_hits += len(gs)
        hole_geoms.extend(gs)
        dias = sorted({round(d, 3) for _, _, d in ex.hits})
        log(f"      钻孔 {os.path.basename(dp)}: {len(ex.hits)} 孔 + "
            f"{len(ex.slots)} 槽，刀具 {len(dias)} 种，"
            f"直径 {dias[:8]}{'...' if len(dias) > 8 else ''} mm")

    log(f"\n[3/6] 打穿孔屏蔽：{n_hits} 个真孔，禁布区外扩 "
        f"{params.hole_clearance:.2f}mm")
    log(f"      ⌀{params.via_dia:.2f}mm 及以下按过孔处理，不参与避让"
        f"（{n_via} 个，盘中孔焊盘因此得以保留）")

    holes = U(hole_geoms) if hole_geoms else EMPTY
    hole_zone = holes.buffer(params.hole_clearance, quad_segs=8) if not holes.is_empty else EMPTY

    # ---- 开孔过滤 ----
    # clip：只把孔位那一块挖掉，焊盘其余部分留着。插件孔上不会有锡膏开孔
    # （锡膏层本来就不含插件焊盘），真正会撞上的是盘中孔——那种焊盘要保留。
    kept, dropped, clipped = [], [], 0
    small, whole = [], []
    for g in flashes + regions:
        gg = g
        if params.aperture_offset:
            gg = gg.buffer(params.aperture_offset, join_style=1, quad_segs=8)
        if gg.is_empty:
            continue
        d = min_dimension(gg)
        if params.min_aperture > 0 and d < params.min_aperture:
            small.append(d)
            continue
        if not hole_zone.is_empty and gg.intersects(hole_zone):
            if params.hole_policy == "drop":
                dropped.append(gg)
                continue
            gg2 = D(gg, hole_zone)
            if gg2.is_empty or gg2.area < gg.area * 0.02:
                dropped.append(gg)           # 孔把焊盘吃光了，没东西可留
                continue
            # 细到印不出来的边不算数：环形焊盘的壁只剩几十微米时，
            # 留着也印不出来，还会把里面那块料围成会掉的孤岛
            parts = [p for p in polys_of(gg2)
                     if params.min_aperture <= 0
                     or min_dimension(p) >= params.min_aperture]
            if len(parts) == 1 and parts[0].area >= gg.area * 0.02:
                clipped += 1
                gg = parts[0]                # 挖掉孔位，焊盘其余部分留着
            else:
                # 挖完碎成好几块、或只剩一条印不出来的细环：硬裁没有意义。
                # 用户要的是"排孔不排焊盘"，这种情况整个焊盘原样保留
                whole.append(gg)
        kept.append(gg)

    log(f"      开孔总数 {len(flashes) + len(regions)}")
    if params.hole_policy == "drop":
        log(f"      因压在插件孔上整孔删除 {len(dropped)} 个")
    else:
        log(f"      被插件孔挖掉孔位、焊盘保留 {clipped} 个")
        if whole:
            log(f"      挖不动、整个焊盘原样保留 {len(whole)} 个"
                f"（孔比焊盘还宽，挖了只剩印不出来的细边）")
            for w in sorted(whole, key=lambda q: q.area)[:8]:
                c = w.representative_point()
                b = w.bounds
                log(f"        {b[2] - b[0]:.2f} x {b[3] - b[1]:.2f} 焊盘 "
                    f"({c.x:.2f}, {c.y:.2f}) 锡膏会印到孔上")
            warn(f"有 {len(whole)} 个焊盘压在插件孔/槽孔上，挖掉孔位就只剩"
                 f"印不出来的细边，按“保留焊盘”原样留着了。"
                 f"这几处锡膏会漏进孔里，介意的话用 --hole-policy drop 整孔删掉")
        if dropped:
            log(f"      挖完不剩什么、整块放弃 {len(dropped)} 个")
    if small:
        log(f"      因小于 {params.min_aperture}mm 剔除 {len(small)} 个"
            f"（最小 {min(small):.3f}mm）")
    if not kept:
        raise JobError("过滤后没有剩下任何开孔，请检查参数（最小开孔 / 孔避让）")

    n_overlap = count_overlaps(kept)
    apertures = snap_geom(U(kept))
    ap_polys = polys_of(apertures)
    dims = sorted(min_dimension(p) for p in ap_polys)
    log(f"      最终开孔 {len(ap_polys)} 个，"
        f"宽度 {dims[0]:.3f} ~ {dims[-1]:.3f} mm")
    if n_overlap:
        log(f"      提示：{n_overlap} 处开孔互相重叠/相切，并集后会有夹点（该处钢网较脆弱）")

    # ---- 板框/治具 ----
    log(f"\n[4/6] 生成几何")
    clr = params.board_clearance
    pocket = snap_geom(board.buffer(clr, join_style=2, quad_segs=16))
    jig_out = snap_geom(pocket.buffer(params.jig_wall, join_style=2, quad_segs=16)) \
        if params.include_jig else pocket

    plate = snap_geom(D(pocket, apertures))
    if plate.is_empty:
        raise JobError("钢网几何为空：开孔把整块板都吃掉了？")
    pieces = polys_of(plate)
    if len(pieces) > 1:
        warn(f"钢网被开孔切成了 {len(pieces)} 块（开孔连成一大片了？）")

    # 治具内壁必须和钢网外沿**顶点完全一致**，否则网格会出现 T 型接点、不闭合。
    # 所以这里用钢网外轮廓反推空腔边界，而不是直接用 board.buffer(间隙)。
    pocket_ring = pocket
    if pieces:
        outer_union = U([Polygon(p.exterior.coords) for p in pieces])
        lp = largest_poly(outer_union)
        if lp is not None and not lp.is_empty:
            pocket_ring = Polygon(lp.exterior.coords)
    ring = D(jig_out, pocket_ring)
    try:
        plate = orient(plate, 1.0)
        ring = orient(ring, 1.0)
    except Exception:
        pass

    tb = params.board_thickness
    ts = params.stencil_thickness
    z_plate1 = tb + ts

    # ---- 分层带（band）模型 ----
    # 关键：治具环与钢网如果各自做成一个拉伸体，两者在空腔边界处共面贴合，
    # 会在共享边上产生非流形（4 个面共用一条边），STL 就不是水密实体。
    # 改成按 Z 分带、每带一张截面，接口面只输出"露出来"的部分，
    # 结果就是**一个闭合流形实体**（切片软件和 CAD 都认）。
    # 注：接口面用 plate / EMPTY 显式给出，保证与邻带的顶点坐标完全一致
    if params.include_jig:
        slab = U([plate, ring])          # 底板整块截面（钢网 + 治具环并集）
        bands = [Band(ring, 0.0, tb, "治具环", top_face=EMPTY, bottom_face=None),
                 Band(slab, tb, z_plate1, "钢网", top_face=None, bottom_face=plate)]
    else:
        bands = [Band(plate, tb, z_plate1, "钢网")]

    ox0, oy0, ox1, oy1 = jig_out.bounds
    n_plate_holes = sum(len(p.interiors) for p in polys_of(plate))
    log(f"      钢网    ：{ts} mm 厚，{len(polys_of(plate))} 块，开孔 {n_plate_holes} 个")
    log(f"      治具    ：壁厚 {params.jig_wall} mm，空腔单边间隙 {clr} mm，"
        f"外形 {ox1 - ox0:.2f} x {oy1 - oy0:.2f} mm")
    log(f"      高度    ：板厚 {tb} → 空腔深 {tb}，总高 {z_plate1:.2f} mm")
    if params.include_jig:
        log(f"      共面检查：治具顶面 z={z_plate1:.3f} 与钢网顶面 z={z_plate1:.3f} -> 共面 OK")

    if params.flip_for_print:
        # 绕 X 轴旋转 180°: (x,y,z) -> (x,-y,-z)，再整体平移回 z>=0；上下颠倒，顶底面互换
        bands = [Band(shapely.affinity.scale(b.poly, yfact=-1.0, origin=(0, 0)),
                      z_plate1 - b.z1, z_plate1 - b.z0, b.name,
                      top_face=(None if b.bottom_face is None else
                                (EMPTY if b.bottom_face.is_empty else
                                 shapely.affinity.scale(b.bottom_face, yfact=-1.0,
                                                        origin=(0, 0)))),
                      bottom_face=(None if b.top_face is None else
                                   (EMPTY if b.top_face.is_empty else
                                    shapely.affinity.scale(b.top_face, yfact=-1.0,
                                                           origin=(0, 0)))))
                 for b in reversed(bands)]
        log(f"      打印姿态：整体翻转 180°（钢网面贴热床），总高 {z_plate1:.2f} mm")
        log(f"      打印分层：0~{ts:.2f} 整块底板(含开孔)，"
              f"{ts:.2f}~{z_plate1:.2f} 只有外圈墙 -> 无悬空、无桥接")

    # ---- 网格 ----
    log(f"\n[5/6] 三角化与网格自检")
    mesh = build_mesh(bands, tol)
    rep = mesh_report(mesh)
    vol_expect = sum(b.poly.area * abs(b.z1 - b.z0) for b in bands)
    vol_dev = abs(rep["volume"] - vol_expect) / max(vol_expect, 1e-9)
    log(f"      网格：{len(mesh)} 三角面，体积 {rep['volume']:.3f} mm³"
        f"（理论 {vol_expect:.3f} mm³，偏差 {vol_dev * 100:.4f}%）")
    log(f"      自检：破面 {rep['open_edges']} 条边 / 非流形 {rep['nonmanifold']} 条边"
        f" / 表面积 {rep['area']:.2f} mm²")
    if rep["open_edges"]:
        warn(f"网格存在 {rep['open_edges']} 条只被 1 个面共享的边（模型不闭合）")
    if vol_dev > 1e-4:
        warn("网格体积与理论值偏差偏大，请检查三角化")
    if rep["nonmanifold"]:
        log(f"      提示：{rep['nonmanifold']} 条非流形边来自开孔相切/重叠处的夹点，"
            f"不影响切片打印，但该处钢网强度较弱")

    res = JobResult(bands=bands, board=board, plate=plate, pocket=pocket,
                    jig_out=jig_out, apertures=apertures, dropped=U(dropped),
                    holes=holes, mesh=mesh)
    res.stats = {
            "board_w": bx1 - bx0, "board_h": by1 - by0,
            "jig_w": ox1 - ox0, "jig_h": oy1 - oy0,
            "n_apertures": len(ap_polys),
            "ap_min": dims[0], "ap_max": dims[-1],
            "n_holes": n_hits, "n_dropped": len(dropped),
            "stencil_t": ts, "board_t": tb, "total_h": z_plate1,
            "triangles": len(mesh), "open_edges": rep["open_edges"],
            "volume": rep["volume"], "elapsed": time.time() - t_start,
            "layer": layer_used, "layer_wanted": params.layer,
            "paste_file": os.path.basename(paste_path),
        }
    log(f"\n[6/6] 完成，用时 {time.time() - t_start:.2f}s")
    return res


def build_mesh(bands: list["Band"], tol: float) -> list:
    """把分层带转成单个闭合三角网格（法线朝外）"""
    mesh: list = []
    for b in bands:
        z0, z1 = min(b.z0, b.z1), max(b.z0, b.z1)
        top = b.poly if b.top_face is None else b.top_face
        bot = b.poly if b.bottom_face is None else b.bottom_face
        n_top = n_bot = 0
        for p in polys_of(top):
            v, idx = triangulate_polygon(p, tol)
            n_top += len(idx)
            mesh.extend(face_triangles(v, idx, z1, up=True))
            cover = tri_coverage(v, idx, p)
            if cover < 0.999:
                warn(f"{b.name} 顶面三角化覆盖 {cover * 100:.3f}%（可能有缺口）")
        for p in polys_of(bot):
            v, idx = triangulate_polygon(p, tol)
            n_bot += len(idx)
            mesh.extend(face_triangles(v, idx, z0, up=False))
            cover = tri_coverage(v, idx, p)
            if cover < 0.999:
                warn(f"{b.name} 底面三角化覆盖 {cover * 100:.3f}%（可能有缺口）")
        walls = wall_triangles(b.poly, z0, z1)
        mesh.extend(walls)
        log(f"      {b.name}: z {z0:.2f}~{z1:.2f}，顶面 {n_top} 三角 / 底面 {n_bot} 三角"
            f" / 侧壁 {len(walls) // 2} 面")
    return mesh


# ============================================================================
# 7. 三角化
# ============================================================================
def _cross2(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


# 坐标都吸附在 1e-6 mm 网格上，垂距小于这个值就当作共线
_GEOM_EPS = 1e-7


def _perp_dist(p, a, b) -> float:
    """p 到直线 ab 的垂距（ab 退化成点时返回点距）"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    return abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / L


def _pt_in_tri(p, a, b, c) -> bool:
    """**严格**内部判定：正好落在三角形边上的点不算挡住。

    边上有点只说明剪出来的那条边会"穿过"这个点（环变成弱简单），面积
    一点不受影响，后面的耳切照样能接着走。要是把它当成挡住，轴对齐的
    矩形／区域填充里几乎每个候选耳都会被否掉，一路卡死到强行剪，
    结果就是三角形重叠、面积对不上。
    """
    d1 = _cross2(a, b, p)
    d2 = _cross2(b, c, p)
    d3 = _cross2(c, a, p)
    return ((d1 > 0 and d2 > 0 and d3 > 0)
            or (d1 < 0 and d2 < 0 and d3 < 0))


def _on_seg(p, a, b) -> bool:
    """p 是否落在线段 ab 的内部（不含端点）"""
    if _perp_dist(p, a, b) > _GEOM_EPS:
        return False
    return (min(a[0], b[0]) - _GEOM_EPS < p[0] < max(a[0], b[0]) + _GEOM_EPS
            and min(a[1], b[1]) - _GEOM_EPS < p[1] < max(a[1], b[1]) + _GEOM_EPS)


def _ear_ok(pts, v: list[int], k: int, dup: set) -> bool:
    """v[k] 能不能当耳朵剪掉：凸角（或共线的退化角）且三角形内没有别的顶点

    dup 是"在环里出现不止一次"的顶点集合（= 孔合并时那条缝的两个端点）。
    这类点不能套用"凸点必定不在耳内"的结论——环在缝处是自接触的，
    凸凹判据会失灵，漏判就会剪出重叠三角形。
    """
    m = len(v)
    ia, ib, ic = v[(k - 1) % m], v[k], v[(k + 1) % m]
    a, b, c = pts[ia], pts[ib], pts[ic]
    if b == a or b == c:
        return True                       # 零长边：剪掉不丢顶点（坐标还在）
    if _perp_dist(b, a, c) < _GEOM_EPS:
        # 共线的退化耳（零面积）。只要没有别的顶点压在三条边上就剪掉——
        # 这个顶点必须留在顶点表里，否则侧壁有、顶面没有 = T 型接点。
        for j in v:
            if j == ia or j == ib or j == ic:
                continue
            p = pts[j]
            if p == a or p == b or p == c:
                continue
            if _on_seg(p, a, b) or _on_seg(p, b, c) or _on_seg(p, a, c):
                return False
        return True
    if _cross2(a, b, c) <= 0:
        return False                      # 凹角，不是耳
    for jj in range(m):
        j = v[jj]
        if j == ia or j == ib or j == ic:
            continue
        p = pts[j]
        if p == a or p == b or p == c:
            continue                      # 缝上的重复顶点，不算挡住
        # 简单多边形里凸点不可能落在耳内部，可以跳过来省判断；
        # 缝上的重复点除外（见 docstring）
        if j not in dup and _cross2(pts[v[jj - 1]], p, pts[v[(jj + 1) % m]]) > 0:
            continue
        if _pt_in_tri(p, a, b, c):
            return False
    return True


def _earclip(pts, ring: list[int]) -> list:
    """耳切：ring 是 CCW 简单多边形的顶点索引，返回三角形索引"""
    v = list(ring)
    if len(v) < 3:
        return []
    seen: dict = {}
    for j in v:
        seen[pts[j]] = seen.get(pts[j], 0) + 1
    dup = {j for j in v if seen[pts[j]] > 1}
    tris: list = []
    guard = 0
    limit = 8 * len(v) * len(v) + 128
    i = 0
    stalled = 0
    while len(v) > 3 and guard < limit:
        guard += 1
        m = len(v)
        i %= m
        if _ear_ok(pts, v, i, dup):
            tris.append((v[(i - 1) % m], v[i], v[(i + 1) % m]))
            v.pop(i)
            stalled = 0
            continue
        i += 1
        stalled += 1
        if stalled < m:
            continue
        # 转满一整圈都剪不动（数值退化）：强行剪掉最"胖"的凸角，保证能推进
        best_k, best_h = -1, 0.0
        for k in range(m):
            a, b, c = pts[v[(k - 1) % m]], pts[v[k]], pts[v[(k + 1) % m]]
            if _cross2(a, b, c) <= 0:
                continue
            h = _perp_dist(b, a, c)
            if h > best_h:
                best_h, best_k = h, k
        if best_k < 0:
            break
        tris.append((v[(best_k - 1) % m], v[best_k], v[(best_k + 1) % m]))
        v.pop(best_k)
        stalled = 0
        i = 0
    if len(v) == 3:
        tris.append((v[0], v[1], v[2]))
    return tris


def _seg_touch(p, q, r1, r2) -> bool:
    """两线段是否相交**或接触**（含共线重叠、端点压边）。

    宁可误报不可漏报：漏报会让缝穿过已有的边，合出来的环自交，
    耳切就会切出重叠三角形（面积对不上、网格破面）。
    """
    d1 = _cross2(p, q, r1)
    d2 = _cross2(p, q, r2)
    d3 = _cross2(r1, r2, p)
    d4 = _cross2(r1, r2, q)
    if (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0):
        return True
    return (_on_seg(r1, p, q) or _on_seg(r2, p, q)
            or _on_seg(p, r1, r2) or _on_seg(q, r1, r2))


def _seg_hits(pts, rings, p: tuple, q: tuple) -> int:
    """线段 p-q 与这些环的边相交/接触的次数（共端点的边不算）"""
    n_hit = 0
    for ring in rings:
        n = len(ring)
        for k in range(n):
            r1 = pts[ring[k]]
            r2 = pts[ring[(k + 1) % n]]
            if r1 == p or r1 == q or r2 == p or r2 == q:
                continue
            if _seg_touch(p, q, r1, r2):
                n_hit += 1
    return n_hit


def _ray_candidates(pts, shell: list[int], m) -> list[int]:
    """从 m 往 -x 打一条射线，返回第一个被命中的壳边两个端点在 shell 里的下标
    （按离 m 的距离从近到远排）。

    射线是"首中"的，所以 m 到命中点这一段必定不穿过任何边；接这条边的
    端点多半也干净——比"挨个试所有顶点"快得多，也准得多。

    必须按距离排：不然每个孔都会挑到同一个角点，几条缝全挤在一个顶点上，
    剪到最后剩一圈张牙舞爪的大三角形，一个耳都剪不出来。
    """
    best_x = None
    edge = None
    n = len(shell)
    for k in range(n):
        r1, r2 = pts[shell[k]], pts[shell[(k + 1) % n]]
        if r1[1] == r2[1]:
            continue                      # 水平边和射线平行
        lo, hi_ = (r1, r2) if r1[1] < r2[1] else (r2, r1)
        if not (lo[1] <= m[1] <= hi_[1]):
            continue
        x = lo[0] + (m[1] - lo[1]) * (hi_[0] - lo[0]) / (hi_[1] - lo[1])
        if x > m[0]:
            continue
        if best_x is None or x > best_x:  # 取 x 最大的 = 往左最先撞到的
            best_x, edge = x, (k, (k + 1) % n)
    if edge is None:
        return []
    return sorted(edge, key=lambda k: (pts[shell[k]][0] - m[0]) ** 2
                                 + (pts[shell[k]][1] - m[1]) ** 2)


def _bridge_hole(pts, shell: list[int], hole: list[int],
                 others: list[list[int]]) -> list:
    """把孔用一条"零宽缝"接到外环上（耳切法的标准前置步骤）。

    缝不能穿过任何一条边，否则合出来的环不是简单多边形，耳切会切出重叠
    三角形。先试射线首中边的端点（快、几乎总是干净的），都不行才按距离
    扫全部壳顶点；实在全被挡住就退而求其次挑挡得最少的那个。
    """
    rings = [shell, hole] + others
    n_shell = len(shell)

    def pick(m, cand):
        """挑一个不挡路的；都挡路就挑挡得最少的"""
        best = None
        for k in cand:
            n = _seg_hits(pts, rings, m, pts[shell[k]])
            if n == 0:
                return (0, k)
            if best is None or n < best[0]:
                best = (n, k)
        return best

    found = None
    # 1) 便宜的路子：从孔的每个顶点往左打射线，只验首中边的两个端点
    for hi in sorted(range(len(hole)),
                     key=lambda k: (pts[hole[k]][0], pts[hole[k]][1])):
        m = pts[hole[hi]]
        r = pick(m, _ray_candidates(pts, shell, m))
        if r and r[0] == 0:
            found = (r[1], hi)
            break
    # 2) 射线法都不干净：按距离扫全部壳顶点（贵，但兜得住）
    if found is None:
        hi = max(range(len(hole)), key=lambda k: (pts[hole[k]][0], -pts[hole[k]][1]))
        m = pts[hole[hi]]
        cand = sorted(range(n_shell),
                      key=lambda k: (pts[shell[k]][0] - m[0]) ** 2
                                    + (pts[shell[k]][1] - m[1]) ** 2)
        found = (pick(m, cand)[1], hi)
    si, hi = found
    h_rot = hole[hi:] + hole[:hi]         # 从缝的孔侧端点开始，保持孔的绕向
    return shell[:si + 1] + h_rot + [hole[hi], shell[si]] + shell[si + 1:]


def _pack_tris(pts, tris):
    """三角形索引 -> (去重后的顶点表, 三角形表)"""
    index: dict = {}
    verts: list = []
    out: list = []
    for t in tris:
        row = []
        for k in t:
            key = pts[k]
            if key not in index:
                index[key] = len(verts)
                verts.append(key)
            row.append(index[key])
        if len(set(row)) == 3:
            out.append(tuple(row))
    return verts, out


def _earcut_with_holes(poly: Polygon):
    """耳切三角化带孔多边形。只在原顶点上切分、不新增顶点，
    所以顶面和侧壁的顶点集天然一致（Delaunay 补 Steiner 点会破坏这一点）。

    共线顶点会切出一个零面积三角形——这是故意的：那个顶点必须留在顶点表
    里，否则侧壁有、顶面没有，接缝处就是 T 型接点（破面）。

    孔合并的顺序会影响成败（缝互相挡路），所以按几种常见顺序各试一遍，
    取面积最贴合的；第一遍就完美的话直接返回。
    """
    poly = orient(poly, 1.0)              # 外环 CCW、内环 CW
    pts: list = []
    rings: list[list[int]] = []
    for r in [poly.exterior] + list(poly.interiors):
        cs = list(r.coords)[:-1]
        if len(cs) < 3:
            continue
        base = len(pts)
        pts.extend(cs)
        rings.append(list(range(base, base + len(cs))))
    if not rings:
        return [], []
    shell0, holes = rings[0], rings[1:]

    orders = [list(range(len(holes)))]
    if len(holes) > 1:
        orders.append(sorted(range(len(holes)),
                             key=lambda i: (min(pts[k][0] for k in holes[i]),
                                            min(pts[k][1] for k in holes[i]))))
        orders.append(sorted(range(len(holes)),
                             key=lambda i: (-max(pts[k][0] for k in holes[i]),
                                            -max(pts[k][1] for k in holes[i]))))

    best = ([], [], -1.0)
    for order in orders:
        seq = [holes[i] for i in order]
        shell = shell0
        for k, h in enumerate(seq):
            shell = _bridge_hole(pts, shell, h, seq[k + 1:])
        verts, out = _pack_tris(pts, _earclip(pts, shell))
        if not out:
            continue
        cov = tri_coverage(verts, out, poly)
        if abs(cov - 1.0) < 1e-6:         # 严丝合缝，不用再试
            return verts, out
        if abs(cov - 1.0) < abs(best[2] - 1.0):
            best = (verts, out, cov)
    return best[0], best[1]


def _boundary_sets_match(poly: Polygon, verts) -> bool:
    """三角化的边界顶点集是否和多边形**完全一致**。

    多一个点（Steiner）→ 顶面被细分、侧壁没细分；少一个点 → 侧壁有、顶面没有。
    两种情况都会在接缝处留下 T 型接点 = 破面。
    """
    want = set()
    for r in [poly.exterior] + list(poly.interiors):
        want |= set(list(r.coords)[:-1])
    got = set(verts)
    if not want <= got:
        return False                                  # 边界上丢点了
    extra = list(got - want)
    if not extra:
        return True
    try:                                              # 多出来的点不能在边界上
        d = shapely.distance(shapely.points(extra), poly.boundary)
        return not bool((d < 1e-7).any())
    except Exception:
        return all(poly.boundary.distance(Point(p)) >= 1e-7 for p in extra)


def _tris_to_index(tri_geoms, poly: Polygon):
    """shapely 三角形 -> (顶点表, 索引表)。三角形统一转成逆时针。"""
    index: dict = {}
    verts: list = []
    tris: list = []
    for t in tri_geoms:
        c = list(t.exterior.coords)[:3]
        if signed_area(c) < 0:
            c = [c[0], c[2], c[1]]
        idx = []
        for p in c:
            if p not in index:
                index[p] = len(verts)
                verts.append(p)
            idx.append(index[p])
        if len(set(idx)) == 3:
            tris.append(tuple(idx))
    return verts, tris


def triangulate_polygon(poly: Polygon, tol: float, max_rounds: int = 4):
    """带孔多边形三角化，返回 [(x,y)] 顶点表 + [(i,j,k)] 索引（逆时针）。

    主路是 GEOS 的**约束** Delaunay（constrained_delaunay_triangles）：
    它把多边形的边当成约束边，三角形绝不会横跨一个孔，而且**不添 Steiner 点**，
    顶点表 = 原多边形顶点表，和侧壁天然对得上（老的点集 Delaunay 两样都做不到：
    跨孔的三角形会被 covers 过滤掉、留下细缝，补缝时又要往边界塞点 -> T 型接点）。
    只在多边形本身不合法（自交等）时它才抛异常。

    退路是耳切法：慢、三角形可能细长，但不依赖 GEOS 的有效性检查，
    合法性判据一样严格（覆盖率 + 边界顶点集），不达标宁可报警也不用。
    """
    poly = orient(poly, 1.0)
    want = set()
    for r in [poly.exterior] + list(poly.interiors):
        want |= set(list(r.coords)[:-1])
    if len(want) < 3:
        return [], []

    cdt = getattr(shapely, "constrained_delaunay_triangles", None)
    if cdt is not None:
        try:
            verts, tris = _tris_to_index(polys_of(cdt(poly)), poly)
            # 判据必须卡到"严丝合缝"：漏掉的往往只是贴着边的一条细缝，
            # 面积上只有万分之几，网格上却是实打实的破面。
            if tris and _boundary_sets_match(poly, verts) \
                    and tri_coverage(verts, tris, poly) > 1.0 - 1e-9:
                return verts, tris
        except Exception as exc:
            warn(f"约束 Delaunay 失败（{type(exc).__name__}: {exc}），改用耳切法")

    ev, et = _earcut_with_holes(poly)
    if et:
        return ev, et
    warn("三角化失败：多边形可能自交或退化")
    return [], []


def face_triangles(verts, idx, z: float, up: bool = True) -> list:
    """把平面三角化结果放到高度 z；up=True 法线朝 +Z"""
    out = []
    for a, b, c in idx:
        pa = (verts[a][0], verts[a][1], z)
        pb = (verts[b][0], verts[b][1], z)
        pc = (verts[c][0], verts[c][1], z)
        out.append((pa, pb, pc) if up else (pa, pc, pb))
    return out


def wall_triangles(poly, z0: float, z1: float) -> list:
    """侧面墙：沿外环(逆时针)与内环(顺时针)生成四边形，法线朝外。
    poly 可能是 MultiPolygon（开孔把料切成几块了），逐块处理。"""
    out = []
    for part in polys_of(poly):
        poly_o = orient(part, 1.0)
        for r in [poly_o.exterior] + list(poly_o.interiors):
            cs = list(r.coords)[:-1]
            m = len(cs)
            for i in range(m):
                x1, y1 = cs[i]
                x2, y2 = cs[(i + 1) % m]
                if abs(x1 - x2) < 1e-9 and abs(y1 - y2) < 1e-9:
                    continue
                v00 = (x1, y1, z0)
                v10 = (x2, y2, z0)
                v11 = (x2, y2, z1)
                v01 = (x1, y1, z1)
                out.append((v00, v10, v11))
                out.append((v00, v11, v01))
    return out


def tri_coverage(verts, idx, poly: Polygon) -> float:
    """顶面三角面积之和 / 多边形面积，用于判断三角化有没有漏面"""
    if not idx or poly.area <= 0:
        return 0.0
    a = 0.0
    for i, j, k in idx:
        (x1, y1), (x2, y2), (x3, y3) = verts[i], verts[j], verts[k]
        a += abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2.0
    return a / poly.area


def mesh_report(mesh) -> dict:
    """网格自检：体积（散度定理）+ 非流形边统计"""
    vol = 0.0
    area = 0.0
    edges: dict[tuple, int] = {}
    for a, b, c in mesh:
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        cx, cy, cz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        area += 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz)
        for p, q in ((a, b), (b, c), (c, a)):
            k = (p, q) if p <= q else (q, p)
            edges[k] = edges.get(k, 0) + 1
    # count==1 -> 真正的破面；count>2 -> 闭合但非流形（例如两个开孔相切形成的夹点）
    open_edges = sum(1 for v in edges.values() if v == 1)
    nonmanifold = sum(1 for v in edges.values() if v > 2)
    return {"volume": vol, "area": area, "open_edges": open_edges,
            "nonmanifold": nonmanifold, "edges": len(edges)}


# ============================================================================
# 8. 导出：STL
# ============================================================================
def write_stl(path: str, mesh) -> float:
    with open(path, "wb") as f:
        f.write(b"PCB stencil + jig (function.py)".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(mesh)))
        for a, b, c in mesh:
            ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
            vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz)
            if ln > 1e-15:
                nx, ny, nz = nx / ln, ny / ln, nz / ln
            f.write(struct.pack("<3f", nx, ny, nz))
            for v in (a, b, c):
                f.write(struct.pack("<3f", v[0], v[1], v[2]))
            f.write(struct.pack("<H", 0))
    return os.path.getsize(path)


# ============================================================================
# 9. 导出：STEP (AP214, FACETED_BREP，纯手写，不依赖 OpenCASCADE)
# ============================================================================
def write_step(path: str, mesh, title="PCB_Stencil"):
    """把三角网格写成一个 STEP 实体（AP214 / FACETED_BREP + POLY_LOOP）。

    选择 FACETED_BREP 的原因：不需要 EDGE_CURVE / VERTEX 拓扑，每个三角面
    独立自洽，写法简单、各种 CAD 都能打开，而且同样逻辑可以直接翻译成 C++，
    不必拖上 OpenCASCADE 这种几百 MB 的依赖。
    """
    ctx = {"id": 0, "body": []}

    def add(text: str) -> int:
        ctx["id"] += 1
        ctx["body"].append(f"#{ctx['id']}={text};")
        return ctx["id"]

    def pt(x, y, z, cache={}):
        key = (round(x, 5), round(y, 5), round(z, 5))
        if key in cache:
            return cache[key]
        i = add(f"CARTESIAN_POINT('',({x:.6f},{y:.6f},{z:.6f}))")
        cache[key] = i
        return i

    plane_cache = {}
    dir_cache = {}

    def direction(v):
        key = (round(v[0], 6), round(v[1], 6), round(v[2], 6))
        if key in dir_cache:
            return dir_cache[key]
        i = add(f"DIRECTION('',({v[0]:.9f},{v[1]:.9f},{v[2]:.9f}))")
        dir_cache[key] = i
        return i

    def plane_for(normal, origin):
        """按 法线+平面到原点距离 构造 PLANE（AXIS2_PLACEMENT_3D：z=法线, x=参考方向）

        同一个平面上的所有三角面共用同一个 PLANE 实体，STEP 体积能小一大截。
        """
        n = normal
        d = n[0] * origin[0] + n[1] * origin[1] + n[2] * origin[2]
        key = (round(n[0], 6), round(n[1], 6), round(n[2], 6), round(d, 5))
        if key in plane_cache:
            return plane_cache[key]
        if abs(n[2]) > 0.9:
            rx = (1.0, 0.0, 0.0)
        else:
            rx = (n[1], -n[0], 0.0)
            ln = math.hypot(rx[0], rx[1]) or 1.0
            rx = (rx[0] / ln, rx[1] / ln, 0.0)
        # 平面上离世界原点最近的点作为放置原点，保证同一平面得到同一个实体
        o = pt(n[0] * d, n[1] * d, n[2] * d)
        ax = add(f"AXIS2_PLACEMENT_3D('',#{o},#{direction(n)},#{direction(rx)})")
        pid = add(f"PLANE('',#{ax})")
        plane_cache[key] = pid
        return pid

    faces = []
    for a, b, c in mesh:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        ln = math.sqrt(nx * nx + ny * ny + nz * nz)
        if ln < 1e-15:
            continue
        n = (nx / ln, ny / ln, nz / ln)
        pl = plane_for(n, a)
        p1, p2, p3 = pt(*a), pt(*b), pt(*c)
        loop = add(f"POLY_LOOP('',(#{p1},#{p2},#{p3}))")
        bound = add(f"FACE_OUTER_BOUND('',#{loop},.T.)")
        faces.append(add(f"ADVANCED_FACE('',(#{bound}),#{pl},.T.)"))
    if not faces:
        raise JobError("STEP 导出失败：没有生成任何面")
    shell = add("CLOSED_SHELL('',(" + ",".join(f"#{f}" for f in faces) + "))")
    shells = [add(f"FACETED_BREP('{title}',#{shell})")]

    ax_pt = pt(0.0, 0.0, 0.0)
    world = add(f"AXIS2_PLACEMENT_3D('',#{ax_pt},#{direction((0, 0, 1))},#{direction((1, 0, 0))})")
    items = ",".join(f"#{s}" for s in shells) + f",#{world}"

    # 前置头部实体
    head = []
    hid = {"n": 0}

    def hadd(text):
        hid["n"] += 1
        head.append(f"#{hid['n']}={text};")
        return hid["n"]

    ac = hadd("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    hadd(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{ac})")
    pc = hadd(f"PRODUCT_CONTEXT('',#{ac},'mechanical')")
    prod = hadd(f"PRODUCT('{title}','{title}','',(#{pc}))")
    pdf = hadd(f"PRODUCT_DEFINITION_FORMATION('','',#{prod})")
    pdc = hadd(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{ac},'design')")
    pd = hadd(f"PRODUCT_DEFINITION('design','',#{pdf},#{pdc})")
    pds = hadd(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
    lu = hadd("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    au = hadd("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    sa = hadd("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = hadd(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{lu},"
               "'distance_accuracy_value','')")
    gc = hadd("(GEOMETRIC_REPRESENTATION_CONTEXT(3)"
              f"GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))"
              f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{lu},#{au},#{sa}))"
              "REPRESENTATION_CONTEXT('',''))")

    # 头部实体编号与 body 冲突：统一整体重编号（把 head 放前面）
    offset = len(head)
    body_lines = []
    for ln in ctx["body"]:
        m = re.match(r"#(\d+)=(.*);$", ln, re.S)
        nid = int(m.group(1)) + offset
        rest = re.sub(r"#(\d+)", lambda mm: "#" + str(int(mm.group(1)) + offset),
                      m.group(2))
        body_lines.append(f"#{nid}={rest};")
    # head 内部引用也要 +0（本来就是自己的编号），但 items 里的引用需要 +offset
    items_shifted = re.sub(r"#(\d+)", lambda mm: "#" + str(int(mm.group(1)) + offset), items)
    shape = f"#{offset + ctx['id'] + 1}=" \
            f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({items_shifted}),#{gc + offset});"
    sdr = f"#{offset + ctx['id'] + 2}=SHAPE_DEFINITION_REPRESENTATION(#{pds},#{offset + ctx['id'] + 1});"

    stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    out = [
        "ISO-10303-21;",
        "HEADER;",
        f"FILE_DESCRIPTION(('PCB stencil and jig'),'2;1');",
        f"FILE_NAME('{os.path.basename(path)}','{stamp}',(''),(''),"
        f"'function.py','PCB workshop','');",
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));",
        "ENDSEC;",
        "DATA;",
    ]
    out.extend(head)
    out.extend(body_lines)
    out.append(shape)
    out.append(sdr)
    out.append("ENDSEC;")
    out.append("END-ISO-10303-21;")
    with open(path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(out) + "\n")
    return os.path.getsize(path)


# ============================================================================
# 10. 预览：3D HTML (three.js)
# ============================================================================
def write_html(path: str, mesh, title: str) -> None:
    import struct as _s
    buf = bytearray()
    buf += b"".ljust(80, b"\0")
    buf += _s.pack("<I", len(mesh))
    for a, b, c in mesh:
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        buf += _s.pack("<3f", nx / ln, ny / ln, nz / ln)
        for v in (a, b, c):
            buf += _s.pack("<3f", v[0], v[1], v[2])
        buf += _s.pack("<H", 0)
    b64 = base64.b64encode(bytes(buf)).decode()
    html = f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8"><title>{title}</title>
<style>
 body{{margin:0;background:#1b1e24;color:#ddd;font:13px/1.6 system-ui,sans-serif}}
 #tip{{position:fixed;left:12px;top:10px;z-index:9;background:#0009;padding:8px 12px;border-radius:6px}}
 #err{{position:fixed;left:50%;top:50%;transform:translate(-50%,-50%);display:none;
      background:#3a1d1d;padding:16px 22px;border-radius:8px}}
</style></head><body>
<div id="tip"><b>{title}</b><br>左键旋转 · 右键平移 · 滚轮缩放</div>
<div id="err">无法加载 three.js（需要联网）。<br>请直接打开同目录的 STL 文件查看。</div>
<script type="importmap">{{"imports":{{
 "three":"https://unpkg.com/three@0.160.0/build/three.module.js",
 "three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}}}</script>
<script type="module">
import * as THREE from 'three';
import {{OrbitControls}} from 'three/addons/controls/OrbitControls.js';
const bin = atob("{b64}");
const buf = new Uint8Array(bin.length);
for (let i=0;i<bin.length;i++) buf[i]=bin.charCodeAt(i);
const dv = new DataView(buf.buffer);
const n = dv.getUint32(80, true);
const pos = new Float32Array(n*9), nor = new Float32Array(n*9);
let o=84;
for (let i=0;i<n;i++){{
  nor.set([dv.getFloat32(o,true),dv.getFloat32(o+4,true),dv.getFloat32(o+8,true)], i*9);
  o+=12;
  for(let k=0;k<3;k++){{ pos.set([dv.getFloat32(o,true),dv.getFloat32(o+4,true),dv.getFloat32(o+8,true)], i*9+k*3); o+=12; }}
  o+=2;
}}
const g = new THREE.BufferGeometry();
g.setAttribute('position', new THREE.BufferAttribute(pos,3));
g.setAttribute('normal', new THREE.BufferAttribute(nor,3));
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1b1e24);
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const d1 = new THREE.DirectionalLight(0xffffff, 0.9); d1.position.set(1,1.4,1); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.5); d2.position.set(-1,-1,0.6); scene.add(d2);
const mat = new THREE.MeshStandardMaterial({{color:0x59c2ff, metalness:0.15, roughness:0.6,
   side:THREE.DoubleSide, flatShading:true}});
const mesh = new THREE.Mesh(g, mat); scene.add(mesh);
g.computeBoundingBox();
const c = g.boundingBox.getCenter(new THREE.Vector3());
mesh.position.sub(c);
const size = g.boundingBox.getSize(new THREE.Vector3()).length();
scene.add(new THREE.GridHelper(size*1.6, 20, 0x334, 0x223));
const cam = new THREE.PerspectiveCamera(45, innerWidth/innerHeight, 0.1, size*40);
cam.position.set(size*0.7, -size*0.9, size*0.8);
const r = new THREE.WebGLRenderer({{antialias:true}}); r.setSize(innerWidth, innerHeight);
document.body.appendChild(r.domElement);
const ctl = new OrbitControls(cam, r.domElement); ctl.target.set(0,0,0);
addEventListener('resize', ()=>{{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();
  r.setSize(innerWidth,innerHeight);}});
(function loop(){{requestAnimationFrame(loop);ctl.update();r.render(scene,cam);}})();
</script>
<script>window.addEventListener('error',()=>{{document.getElementById('err').style.display='block';}});</script>
</body></html>"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)


# ============================================================================
# 11. 预览：2D (tkinter canvas 绘制，GUI 与导出共用)
# ============================================================================
def make_section(result: JobResult, n_bands: int = 1):
    """返回剖切示意 [(x0,x1,z0,z1,kind)]，用于验证 z 向叠层与共面"""
    out = []
    if result.board is None:
        return out
    bx0, by0, bx1, by1 = result.board.bounds
    ymid = (by0 + by1) / 2.0
    cut = box(bx0 - 50, ymid - 0.05, bx1 + 50, ymid + 0.05)
    for b in result.bands:
        inter = b.poly.intersection(cut)
        for p in polys_of(inter):
            x0, _, x1, _ = p.bounds
            out.append((x0, x1, min(b.z0, b.z1), max(b.z0, b.z1), b.name))
    return out


# ============================================================================
# 12. 自检样例
# ============================================================================
SELFTEST_PASTE = """G04 selftest paste*
%FSLAX46Y46*%
%MOMM*%
%LPD*%
G01*
%ADD10C,0.500000*%
%ADD11R,1.200000X0.900000*%
%ADD12O,1.600000X0.800000*%
%ADD13C,0.300000*%
%AMRoundRect*1,1,$1,$2,$3*1,1,$1,$4,$5*1,1,$1,0-$2,0-$3*1,1,$1,0-$4,0-$5*20,1,$1,$2,$3,$4,$5,0*20,1,$1,$4,$5,0-$2,0-$3,0*20,1,$1,0-$2,0-$3,0-$4,0-$5,0*20,1,$1,0-$4,0-$5,$2,$3,0*4,1,4,$2,$3,$4,$5,0-$2,0-$3,0-$4,0-$5,$2,$3,0*%
%AMOval*1,1,$1,$2,$3*1,1,$1,$4,$5*20,1,$1,$2,$3,$4,$5,0*%
%ADD14RoundRect,0.1X-0.35X0.15X0.35X0.15*%
%ADD15Oval,0.3X0X-0.4X0X0.4*%
D14*
X5000000Y12000000D03*
X5600000Y12000000D03*
D15*
X8000000Y12000000D03*
D10*
X5000000Y5000000D03*
X5500000Y5000000D03*
X6000000Y5000000D03*
X6500000Y5000000D03*
D11*
X5000000Y7000000D03*
X7000000Y7000000D03*
D12*
X5000000Y9000000D03*
D13*
X9000000Y5000000D03*
X9500000Y5000000D03*
G36*
X20000000Y20000000D02*
X20000000Y22000000D01*
X22000000Y22000000D01*
X22000000Y20000000D01*
X20000000Y20000000D01*
G37*
M02*
"""

SELFTEST_OUTLINE = """G04 selftest outline*
%FSLAX46Y46*%
%MOMM*%
%ADD10C,0.100000*%
D10*
G01*
X0Y0D02*
X30000000Y0D01*
X30000000Y20000000D01*
X0Y20000000D01*
X0Y0D01*
M02*
"""

SELFTEST_DRILL = """M48
;FILE_FORMAT=4:4
METRIC,TZ
T1C0.800
T2C1.000
T3C3.200
%
G90
G05
T1
X5.080Y7.620
X7.620Y7.620
X10.160Y7.620
X12.700Y7.620
T2
X2.540Y2.540
X27.460Y2.540
T3
X1.500Y18.500
X28.500Y18.500
M30
"""


def selftest(workdir: str | None = None) -> JobResult:
    d = workdir or os.path.join(tempfile.gettempdir(), "stencil_selftest")
    os.makedirs(d, exist_ok=True)
    files = {
        "demo-TopPaste.GTP": SELFTEST_PASTE,
        "demo-BoardOutline.GKO": SELFTEST_OUTLINE,
        "demo-PTH.drl": SELFTEST_DRILL,
    }
    for fn, content in files.items():
        with open(os.path.join(d, fn), "w", encoding="utf-8") as f:
            f.write(content)
    log(f"自检样例目录：{d}")
    p = Params(out_dir=os.path.join(d, "out"))
    return run_job(d, p)


# ============================================================================
# 13. 导出入口
# ============================================================================
def export_all(result: JobResult, params: Params, base_name: str = "stencil") -> dict:
    out_dir = params.out_dir or os.path.join(os.getcwd(), "output")
    os.makedirs(out_dir, exist_ok=True)
    base = f"{base_name}_t{params.stencil_thickness:g}_b{params.board_thickness:g}"

    stl_path = os.path.join(out_dir, base + ".stl")
    sz = write_stl(stl_path, result.mesh)
    log(f"  STL : {stl_path}  ({sz / 1048576:.2f} MB)")

    step_path = os.path.join(out_dir, base + ".step")
    ssz = write_step(step_path, result.mesh, title=base)
    log(f"  STEP: {step_path}  ({ssz / 1048576:.2f} MB)")

    out = {"stl": stl_path, "step": step_path}
    if params.make_html:
        html_path = os.path.join(out_dir, base + "_preview.html")
        write_html(html_path, result.mesh, base)
        out["html"] = html_path
        log(f"  HTML: {html_path}")
    return out


def default_out_dir(input_path: str) -> str:
    """默认输出目录：目录输入 -> 该目录下 stencil_out；文件/zip -> 同级 stencil_out"""
    p = os.path.abspath(input_path or ".")
    base = p if os.path.isdir(p) else os.path.dirname(p)
    return os.path.join(base or os.getcwd(), "stencil_out")


def open_dir(path: str) -> None:
    """用系统文件管理器打开目录"""
    if not path:
        raise JobError("还没有输出目录")
    path = os.path.abspath(path)
    if not os.path.isdir(path):
        raise JobError(f"目录不存在: {path}")
    if hasattr(os, "startfile"):
        os.startfile(path)                                  # Windows
    elif sys.platform == "darwin":
        subprocess.Popen(["open", path])
    else:
        subprocess.Popen(["xdg-open", path])


# ============================================================================
# 14. GUI
# ============================================================================
def run_gui(smoke_input: str | None = None):  # pragma: no cover
    """smoke_input 非空时做无人工干预的自检：自动跑一遍预览+导出后关窗。"""
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    frozen = getattr(sys, "frozen", False)
    root = tk.Tk()
    root.title(APP_TITLE + ("   v" + APP_VERSION if frozen else "   (源码运行)"))
    root.geometry("1400x900")

    state = {"result": None, "params": Params()}
    P = state["params"]

    def on_option_change():
        """单选/勾选一点就重算，不用再点一次预览（前提是之前跑通过）"""
        if state["result"] is not None:
            do_preview(silent=True)

    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")

    # --- 输入 ---
    ttk.Label(top, text="PCB 文件（目录 / zip / 单个文件）:").grid(row=0, column=0, sticky="w")
    var_in = tk.StringVar(value="")
    e_in = ttk.Entry(top, textvariable=var_in, width=80)
    e_in.grid(row=0, column=1, columnspan=6, sticky="we", padx=4)
    e_in.bind("<Return>", lambda _e: do_preview())
    def browse_dir():
        cur = var_in.get().strip()
        init = cur if os.path.isdir(cur) else (os.path.dirname(cur) or os.getcwd())
        d = filedialog.askdirectory(title="选择 Gerber 所在目录", initialdir=init)
        if d:
            var_in.set(os.path.normpath(d))

    def browse_file():
        cur = var_in.get().strip()
        init = cur if os.path.isdir(cur) else (os.path.dirname(cur) or os.getcwd())
        PCB_TYPES = [
            ("PCB 文件（zip / Gerber / 钻孔）",
             "*.zip *.gbr *.ger *.gtp *.gbp *.gts *.gbs *.gto *.gbo *.gko *.gm1 "
             "*.gml *.art *.pho *.drl *.ncd *.xln *.txt"),
            ("压缩包", "*.zip"),
            ("Gerber", "*.gbr *.ger *.gtp *.gbp *.gts *.gbs *.gto *.gbo *.gko *.gm1 *.gml *.art *.pho"),
            ("钻孔 (Excellon)", "*.drl *.ncd *.xln *.txt"),
            ("所有文件", "*.*"),
        ]
        f = filedialog.askopenfilename(title="选择 zip 或单个 Gerber/钻孔文件",
                                       initialdir=init, filetypes=PCB_TYPES)
        if f:
            var_in.set(os.path.normpath(f))

    ttk.Button(top, text="选目录...", command=browse_dir).grid(row=0, column=7, padx=2)
    ttk.Button(top, text="选文件/zip...",
               command=browse_file).grid(row=0, column=8, padx=2)
    ttk.Button(top, text="识别",
               command=lambda: do_scan()).grid(row=0, column=9, padx=2)

    # --- 输出目录（留空 = 自动：目录输入放目录内，文件/zip 放同级）---
    ttk.Label(top, text="输出目录（留空=自动）:").grid(row=1, column=0, sticky="w")
    var_out = tk.StringVar(value="")
    ttk.Entry(top, textvariable=var_out, width=80).grid(row=1, column=1, columnspan=6,
                                                        sticky="we", padx=4)
    ttk.Button(top, text="选目录...",
               command=lambda: var_out.set(
                   os.path.normpath(filedialog.askdirectory(title="选择输出目录")
                                    or var_out.get()))
               ).grid(row=1, column=7, padx=2, columnspan=2)
    def open_out():
        try:
            open_dir(var_out.get().strip() or P.out_dir
                     or default_out_dir(var_in.get().strip()))
        except JobError as e:
            messagebox.showinfo("提示", str(e))

    ttk.Button(top, text="打开", command=open_out).grid(row=1, column=9, padx=2)

    # --- 参数 ---
    fields_spec = [
        ("钢网厚度 mm", "stencil_thickness"),
        ("板厚 mm", "board_thickness"),
        ("治具壁厚 mm", "jig_wall"),
        ("板框间隙 mm", "board_clearance"),
        ("孔避让 mm", "hole_clearance"),
        ("过孔阈值 mm", "via_dia"),
        ("最小开孔 mm", "min_aperture"),
        ("开孔补偿 mm", "aperture_offset"),
        ("圆弧精度 mm", "arc_tolerance"),
    ]
    var_map = {}
    for i, (label, key) in enumerate(fields_spec):
        col = (i % 4) * 2
        row = 2 + i // 4
        ttk.Label(top, text=label).grid(row=row, column=col, sticky="e", padx=(8, 2), pady=2)
        v = tk.StringVar(value=str(getattr(P, key)))
        var_map[key] = v
        e = ttk.Entry(top, textvariable=v, width=10)
        e.grid(row=row, column=col + 1, sticky="w", pady=2)
        e.bind("<Return>", lambda _e: on_option_change())

    opt = ttk.Frame(root, padding=(8, 0))
    opt.pack(fill="x")
    var_hole = tk.StringVar(value=P.hole_policy)
    ttk.Label(opt, text="压在孔上的开孔:").pack(side="left")
    ttk.Radiobutton(opt, text="只挖掉孔位（留焊盘）", variable=var_hole,
                    value="clip", command=on_option_change).pack(side="left")
    ttk.Radiobutton(opt, text="整孔删除", variable=var_hole, value="drop",
                    command=on_option_change).pack(side="left")
    var_layer = tk.StringVar(value="top")
    ttk.Label(opt, text="   钢网面:").pack(side="left")
    ttk.Radiobutton(opt, text="顶层", variable=var_layer, value="top",
                    command=on_option_change).pack(side="left")
    ttk.Radiobutton(opt, text="底层", variable=var_layer, value="bottom",
                    command=on_option_change).pack(side="left")
    var_jig = tk.BooleanVar(value=True)
    var_flip = tk.BooleanVar(value=True)
    var_html = tk.BooleanVar(value=True)
    ttk.Checkbutton(opt, text="  含治具", variable=var_jig,
                    command=on_option_change).pack(side="left")
    ttk.Checkbutton(opt, text="翻转打印姿态", variable=var_flip,
                    command=on_option_change).pack(side="left")
    ttk.Checkbutton(opt, text="生成3D预览HTML", variable=var_html).pack(side="left")
    ttk.Label(opt, text="   （数值参数回车生效）", foreground="#8899aa").pack(side="left")

    btns = ttk.Frame(root, padding=8)
    btns.pack(fill="x")
    btn_preview = ttk.Button(btns, text="① 预览", command=lambda: do_preview())
    btn_preview.pack(side="left")
    btn_run = ttk.Button(btns, text="② 生成并导出 STL + STEP", command=lambda: do_export())
    btn_run.pack(side="left", padx=6)
    ttk.Button(btns, text="打开输出目录",
               command=open_out).pack(side="left", padx=6)
    status = tk.StringVar(value="就绪")
    ttk.Label(btns, textvariable=status).pack(side="left", padx=16)

    mid = ttk.Frame(root)
    mid.pack(fill="both", expand=True, padx=8)
    cv_top = tk.Canvas(mid, bg="#101318", highlightthickness=1,
                       highlightbackground="#333")
    cv_sec = tk.Canvas(mid, bg="#101318", highlightthickness=1, highlightbackground="#333",
                       width=460)
    cv_top.pack(side="left", fill="both", expand=True)
    cv_sec.pack(side="left", fill="both", padx=(6, 0))

    txt = tk.Text(root, height=12, bg="#0d1014", fg="#c8d0d8",
                  font=("Consolas", 9), insertbackground="#ccc")
    txt.pack(fill="both", padx=8, pady=8)

    def collect():
        for k, v in var_map.items():
            try:
                setattr(P, k, float(v.get()))
            except ValueError:
                messagebox.showerror("参数错误", f"参数 {k} 不是数字: {v.get()}")
                raise
        P.hole_policy = var_hole.get()
        P.layer = var_layer.get()
        P.include_jig = var_jig.get()
        P.flip_for_print = var_flip.get()
        P.make_html = var_html.get()

    def do_scan():
        found = scan_input(var_in.get() or ".")
        lines = [f"{k:<13} {os.path.basename(fp)}"
                 for k in sorted(found) for fp in found[k]]
        ok = lambda k: "有" if found.get(k) else "—— 没有 ——"      # noqa: E731
        lines.append("")
        lines.append(f"顶层锡膏(paste_top)   : {ok('paste_top')}")
        lines.append(f"底层锡膏(paste_bottom): {ok('paste_bottom')}")
        lines.append(f"板框(outline)         : {ok('outline')}")
        if found.get("outline_weak"):
            lines.append(f"机械层(仅当板框用)    : {ok('outline_weak')}")
        drills = (found.get("drill", []) + found.get("drill_pth", [])
                  + found.get("drill_npth", []))
        lines.append(f"钻孔(drill)           : {'有' if drills else '—— 没有 ——'}")
        messagebox.showinfo("识别结果", "\n".join(lines) or "没有识别到可用文件")

    def do_preview(silent: bool = False):
        try:
            collect()
            LOG.clear()
            res = run_job(var_in.get(), P)
            state["result"] = res
            draw_top(cv_top, res)
            draw_section(cv_sec, res)
            txt.delete("1.0", "end")
            txt.insert("end", "\n".join(LOG))
            lay_cn = {"top": "顶层", "bottom": "底层"}.get(res.stats.get("layer"), "?")
            if res.stats.get("layer") != res.stats.get("layer_wanted"):
                status.set(f"预览完成：实际用了{lay_cn}！")
            else:
                status.set(f"预览完成：{lay_cn}")
            return True
        except Exception as e:
            state["result"] = None          # 别让旧几何被当成新结果导出去
            txt.delete("1.0", "end")
            txt.insert("end", "\n".join(LOG) + "\n\n" + traceback.format_exc())
            if not silent:
                messagebox.showerror("出错", str(e))
            status.set("出错")
            return False

    def do_export():
        # 每次都按当前参数重算：否则改完参数直接导出，导出的还是上一次的几何
        if not do_preview():
            return
        try:
            # 每次重算，换输入后输出目录跟着走；用户填了就用用户的
            P.out_dir = var_out.get().strip() or default_out_dir(var_in.get())
            files = export_all(state["result"], P, base_name="stencil")
            txt.insert("end", "\n" + "\n".join(f"  -> {v}" for v in files.values()) + "\n")
            status.set("导出完成")
        except Exception as e:
            txt.insert("end", "\n" + traceback.format_exc())
            messagebox.showerror("出错", str(e))

    if smoke_input:
        # 自检模式：不弹窗（弹窗会阻塞），全部走日志
        messagebox.showinfo = lambda *a, **k: log("[GUI] " + " ".join(map(str, a)))
        messagebox.showerror = lambda *a, **k: warn("[GUI] " + " ".join(map(str, a)))
        state["smoke_failed"] = []

        def _smoke():
            var_in.set(smoke_input)
            var_out.set(os.path.join(tempfile.gettempdir(), "stencil_gui_smoke"))
            os.makedirs(var_out.get(), exist_ok=True)
            do_preview()
            if state["result"] is None or not state["result"].bands:
                state["smoke_failed"].append("预览没有得到几何")
            # 回归：改了选项必须重算（旧版会拿上一次的几何去导出）
            n0 = len(polys_of(state["result"].apertures))
            var_layer.set("bottom" if var_layer.get() == "top" else "top")
            on_option_change()
            if state["result"] is None:
                state["smoke_failed"].append("切换钢网面后重算失败")
            else:
                n1 = len(polys_of(state["result"].apertures))
                log(f"[GUI] 切换钢网面 {var_layer.get()} 后开孔数 {n0} -> {n1}")
                if n0 == n1:
                    log("[GUI] 提示：两层开孔数相同（该样例可能只有单面锡膏）")
            do_export()
            log(f"[GUI] 自检结束：{'失败' if state['smoke_failed'] else '预览+导出 OK'}")
            root.destroy()

        root.after(200, _smoke)
    root.mainloop()
    if smoke_input:
        return state["smoke_failed"]


def draw_top(cv, res: JobResult):  # pragma: no cover
    cv.delete("all")
    cv.update_idletasks()
    W = max(cv.winfo_width(), 200)
    H = max(cv.winfo_height(), 200)
    if res.jig_out is None:
        return
    x0, y0, x1, y1 = res.jig_out.bounds
    m = 20
    sc = min((W - 2 * m) / max(x1 - x0, 1e-6), (H - 2 * m) / max(y1 - y0, 1e-6))

    def T(x, y):
        return (m + (x - x0) * sc, H - m - (y - y0) * sc)

    def draw_poly(poly, **kw):
        for p in polys_of(poly):
            c = [T(*pt) for pt in p.exterior.coords]
            cv.create_polygon([v for xy in c for v in xy], **kw)
            for ring in p.interiors:
                c2 = [T(*pt) for pt in ring.coords]
                cv.create_polygon([v for xy in c2 for v in xy], fill="#101318", outline="")

    draw_poly(res.jig_out, fill="#2b3038", outline="#6a7480")
    if res.pocket is not None:
        draw_poly(res.pocket, fill="#101318", outline="#6a7480")
    draw_poly(res.plate, fill="#2f6f9f", outline="#8fc7ff")
    if res.dropped is not None and not res.dropped.is_empty:
        draw_poly(res.dropped, fill="", outline="#ff5555")
    if res.board is not None:
        c = [T(*pt) for pt in res.board.exterior.coords]
        cv.create_polygon([v for xy in c for v in xy], fill="", outline="#ffd479", dash=(4, 3))
    if res.holes is not None and not res.holes.is_empty:
        for p in polys_of(res.holes):
            c = [T(*pt) for pt in p.exterior.coords]
            cv.create_polygon([v for xy in c for v in xy], fill="", outline="#ff5555")
    lay = res.stats.get("layer", "")
    lay_cn = {"top": "顶层", "bottom": "底层"}.get(lay, "?")
    fell_back = lay != res.stats.get("layer_wanted", lay)
    head = (f"钢网面：{lay_cn}   文件：{res.stats.get('paste_file', '?')}   "
            f"开孔 {res.stats.get('n_apertures', '?')} 个   "
            f"板框 {res.stats.get('board_w', 0):.1f}×{res.stats.get('board_h', 0):.1f}mm")
    if fell_back:
        head += "\n← 注意：没有该层数据，实际用了另一层！"
    cv.create_text(10, 10, anchor="nw", fill="#ff9f43" if fell_back else "#8fd18f",
                   text=head, font=("", 10, "bold"))
    cv.create_text(10, 28, anchor="nw", fill="#8899aa", text=(
        "俯视： 蓝=钢网  深灰=治具  黄虚线=板框  红=打穿孔(禁开孔)"), font=("", 9))


def draw_section(cv, res: JobResult):  # pragma: no cover
    cv.delete("all")
    cv.update_idletasks()
    W = max(cv.winfo_width(), 200)
    H = max(cv.winfo_height(), 200)
    sec = make_section(res)
    if not sec:
        return
    x0 = min(s[0] for s in sec)
    x1 = max(s[1] for s in sec)
    z1 = max(s[3] for s in sec)
    m = 30
    sc = min((W - 2 * m) / max(x1 - x0, 1e-6), (H - 2 * m) / max(z1, 1e-6))
    zex = max(3.0, z1 * 1.6)

    def T(x, z):
        return (m + (x - x0) * sc, H - m - z * sc)

    colors = {"钢网": "#2f6f9f", "治具环": "#5a6472"}
    cv.create_line(*T(x0, 0), *T(x1, 0), fill="#556", width=2)
    for sx0, sx1, sz0, sz1, name in sec:
        a = T(sx0, sz0)
        b = T(sx1, sz1)
        cv.create_rectangle(a[0], b[1], b[0], a[1],
                            fill=colors.get(name, "#777"), outline="#aab")
    cv.create_text(10, 10, anchor="nw", fill="#8899aa",
                   text=f"剖面（沿板中心 Y 剖切）  总高 {z1:.2f}mm  "
                        f"钢网 {res.stats.get('stencil_t')}mm  板厚 {res.stats.get('board_t')}mm",
                   font=("", 9))


# ============================================================================
# 15. CLI
# ============================================================================
def build_argparser():
    ap = argparse.ArgumentParser(
        description="PCB(Gerber/Excellon) -> 钢网 + 治具 -> STL/STEP")
    ap.add_argument("-i", "--input", help="输入目录 / zip / 单个 Gerber 文件")
    ap.add_argument("-o", "--out", help="输出目录")
    ap.add_argument("--layer", choices=["top", "bottom"], default="top")
    ap.add_argument("--stencil", type=float, default=0.15, help="钢网厚度 mm")
    ap.add_argument("--board", type=float, default=1.6, help="板厚 mm")
    ap.add_argument("--jig-wall", type=float, default=5.0, help="治具壁厚 mm")
    ap.add_argument("--clearance", type=float, default=0.20, help="板框间隙 mm")
    ap.add_argument("--hole-clearance", type=float, default=0.15, help="打穿孔禁布区外扩 mm")
    ap.add_argument("--hole-policy", choices=["drop", "clip"], default="clip",
                    help="clip=只挖掉孔位、焊盘保留（默认）/ drop=整孔删除")
    ap.add_argument("--via-dia", type=float, default=0.40,
                    help="≤该直径的孔算过孔，不参与避让（默认 0.40）："
                         "盘中孔焊盘要保住")
    ap.add_argument("--min-aperture", type=float, default=0.15)
    ap.add_argument("--offset", type=float, default=0.0, help="开孔补偿 mm")
    ap.add_argument("--arc-tol", type=float, default=0.01)
    ap.add_argument("--no-jig", action="store_true", help="只出钢网，不出治具")
    ap.add_argument("--no-flip", action="store_true", help="不翻转成打印姿态")
    ap.add_argument("--no-html", action="store_true")
    ap.add_argument("--selftest", action="store_true", help="跑内置合成样例自检")
    ap.add_argument("--gui-check", nargs="?", const="", default=None, metavar="目录",
                    help="前端自检：开窗自动跑一遍预览+导出后关闭")
    return ap


def _hide_console_if_frozen() -> None:
    """打包成 exe 后双击运行时，把那个黑框藏起来，只留 GUI 窗口。

    只在"没带命令行参数"（= 打开界面）时调用：带参数跑批处理时黑框要留着，
    不然日志没地方看。源码直接跑时不动，免得调试时控制台莫名其妙消失。
    """
    if not getattr(sys, "frozen", False) or os.name != "nt":
        return
    try:
        import ctypes
        hwnd = ctypes.windll.kernel32.GetConsoleWindow()
        if hwnd:
            ctypes.windll.user32.ShowWindow(hwnd, 0)          # SW_HIDE
    except Exception:
        pass


def main(argv=None):
    setup_console()
    args = build_argparser().parse_args(argv)
    if args.gui_check is not None:
        d = args.gui_check or os.path.join(tempfile.gettempdir(), "stencil_selftest")
        if not os.path.isdir(d):
            selftest()
        bad = run_gui(smoke_input=d)
        log("前端自检：" + ("失败 -> " + "; ".join(bad) if bad else "通过"))
        return 1 if bad else 0
    if args.selftest:
        P = Params(out_dir=args.out or "")
        res = selftest()
        P.out_dir = P.out_dir or os.path.join(tempfile.gettempdir(), "stencil_selftest", "out")
        export_all(res, P)
        return 0
    if not args.input:
        _hide_console_if_frozen()
        run_gui()
        return 0

    P = Params(
        stencil_thickness=args.stencil, board_thickness=args.board,
        jig_wall=args.jig_wall, board_clearance=args.clearance,
        hole_clearance=args.hole_clearance, hole_policy=args.hole_policy,
        via_dia=args.via_dia,
        min_aperture=args.min_aperture, aperture_offset=args.offset,
        arc_tolerance=args.arc_tol, layer=args.layer,
        include_jig=not args.no_jig, flip_for_print=not args.no_flip,
        make_html=not args.no_html,
        out_dir=args.out or default_out_dir(args.input),
    )
    try:
        res = run_job(args.input, P)
        export_all(res, P)
        log("\n完成。建议：用切片软件打开 STL 检查；用 FreeCAD/CAD 打开 STEP 检查。")
    except JobError as e:
        log(f"\n[失败] {e}")
        return 2
    except Exception:
        log("\n[异常]\n" + traceback.format_exc())
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
