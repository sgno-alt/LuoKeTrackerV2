# -*- coding: utf-8 -*-
"""
洛克王国世界 · 小地图实时定位（Windows 版）

移植自 LuoKeMapTracker（Android 版），核心算法保持一致：
    截屏 → 裁剪小地图 → CLAHE 增强 → ORB 特征 → KNN + Lowe 比值筛选
    → RANSAC 求变换 → 小地图中心映射到大地图 → 指数平滑

相对原版的改动：
    1. 截屏/悬浮窗换成 Windows 实现（mss 截屏 + OpenCV 置顶窗口）
    2. 自动跟随游戏窗口，全屏/窗口化切换都能用；每种分辨率单独记一套校准
    3. 大地图分块提取特征（原版整图只取 5000 个点，大地图会稀疏）并缓存
    4. 定位成功后优先在附近区域搜索，更快更稳；失败再全图搜索
    5. 用相似变换代替单应矩阵（小地图没有透视），防止退化解
    6. 小地图可设为圆形并遮掉中心玩家图标，减少干扰特征
    7. 远距离跳变需连续两帧确认，防止误匹配乱跳
    8. 识别小地图中心的玩家箭头，悬浮窗里的箭头跟着角色实际朝向转
       （小地图上北下南固定时，地图本身不转，朝向只能从玩家箭头读出来）

依赖：pip install opencv-python numpy mss pillow
      （Anaconda 用户：双击「创建环境.bat」自动建 luoke 环境并安装）
用法：python luoke_tracker.py              正常启动（首次会进入设置向导）
      python luoke_tracker.py --setup      重新设置（换地图/换窗口）
      python luoke_tracker.py --calibrate  重新校准当前分辨率的小地图位置
"""

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import time
import traceback
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("缺少依赖，请先运行：pip install opencv-python numpy mss pillow")
    input("按回车退出")
    sys.exit(1)

APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "luoke_config.json"
CACHE_DIR = APP_DIR / "cache"
RESOURCES_PATH = APP_DIR / "resources.json"

# OpenCV 窗口标题不支持中文，这里用英文
WIN_NAME = "LuoKe Tracker"
CALIB_WIN = "Select minimap: drag box, Enter=OK, C=cancel"
ARROW_WIN = "Click the player arrow: Enter=OK, F=flip, +/-=range, Esc=skip"
OWN_TITLES = {WIN_NAME, CALIB_WIN, ARROW_WIN}

IS_WIN = platform.system() == "Windows"
CACHE_VERSION = 2

DEFAULT_CONFIG = {
    "map_path": "",
    "mode": "window",            # window = 跟随游戏窗口；monitor = 截整个显示器
    "window_keyword": "洛克王国",
    "monitor": 1,
    "minimap_circle": True,      # 小地图是圆形
    "center_mask_ratio": 0.12,   # 遮掉中心玩家图标的半径（占小地图边长比例）
    "profiles": {},              # {"2560x1440": [x, y, w, h]} 每种分辨率一套校准

    # 匹配参数（与原版对应）
    "interval": 0.1,             # 匹配间隔（秒），原版 0.2
    "map_features": 60000,       # 大地图特征点总数（原版 5000）
    "mini_features": 1500,       # 小地图特征点数
    "fast_threshold": 12,        # 越小特征越多（原版 20）
    "ratio_thresh": 0.75,        # Lowe 比值（同原版）
    "ransac_thresh": 5.0,        # RANSAC 阈值（同原版）
    "min_matches": 8,            # 最少匹配数（同原版）
    "min_inliers": 6,
    "conf_threshold": 0.3,       # 置信度阈值（同原版）
    "smooth_alpha": 0.3,         # 平滑系数（同原版）
    "jump_dist": 500,            # 超过此距离视为跳变（同原版）
    "scale_min": 0.15,           # 小地图→大地图 允许的缩放范围
    "scale_max": 8.0,

    # 朝向箭头（按 A 校准；记下箭头的样子和位置，位置按小地图比例记）
    "arrow": None,
    "arrow_smooth": 0.5,         # 朝向平滑系数，越大转得越跟手
    "arrow_max_rmse": 55.0,      # 和模板的差异超过此值就认为没认出箭头（不更新朝向）

    # 显示
    "view_size": 320,
    "view_zoom": 1.0,
    "window_pos": None,
    "show_resources": True,
}


# ════════════════════════════════════════════════════════════
#  配置
# ════════════════════════════════════════════════════════════

def load_config():
    cfg = dict(DEFAULT_CONFIG)
    if CONFIG_PATH.exists():
        try:
            cfg.update(json.loads(CONFIG_PATH.read_text(encoding="utf-8")))
        except Exception as e:
            print(f"⚠ 配置文件读取失败，使用默认配置：{e}")
    cfg["profiles"] = dict(cfg.get("profiles") or {})
    return cfg


def save_config(cfg):
    CONFIG_PATH.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")


def size_key(w, h):
    return f"{w}x{h}"


def resolve_minimap(cfg, tw, th):
    """按当前画面尺寸取小地图区域；没有精确记录时，用同宽高比的记录等比换算。"""
    profiles = cfg["profiles"]
    k = size_key(tw, th)
    if k in profiles:
        return profiles[k]
    for key, rect in profiles.items():
        try:
            pw, ph = map(int, key.split("x"))
        except ValueError:
            continue
        if abs(pw / ph - tw / th) < 0.01:
            s = tw / pw
            return [int(round(v * s)) for v in rect]
    return None


# ════════════════════════════════════════════════════════════
#  Windows 窗口 / 截屏
# ════════════════════════════════════════════════════════════

if IS_WIN:
    import ctypes
    from ctypes import wintypes

    user32 = ctypes.windll.user32
    _EnumProc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    # 显式声明参数类型，避免 64 位系统上句柄被截断
    for _fn in ("IsWindow", "IsIconic", "IsWindowVisible", "GetWindowTextLengthW"):
        getattr(user32, _fn).argtypes = [wintypes.HWND]
    user32.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    user32.GetClientRect.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.RECT)]
    user32.ClientToScreen.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.POINT)]
    user32.EnumWindows.argtypes = [_EnumProc, wintypes.LPARAM]

    def set_dpi_aware():
        # 让坐标都按物理像素算，否则开了 125%/150% 缩放时截图位置会偏
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            try:
                user32.SetProcessDPIAware()
            except Exception:
                pass

    def client_rect(hwnd):
        """窗口客户区（不含标题栏/边框）在屏幕上的位置 (x, y, w, h)；最小化返回 None。"""
        if not user32.IsWindow(hwnd) or user32.IsIconic(hwnd):
            return None
        r = wintypes.RECT()
        if not user32.GetClientRect(hwnd, ctypes.byref(r)):
            return None
        pt = wintypes.POINT(0, 0)
        if not user32.ClientToScreen(hwnd, ctypes.byref(pt)):
            return None
        w, h = r.right - r.left, r.bottom - r.top
        if w <= 0 or h <= 0:
            return None
        return (pt.x, pt.y, w, h)

    def list_windows():
        result = []

        def cb(hwnd, _):
            if not user32.IsWindowVisible(hwnd):
                return True
            n = user32.GetWindowTextLengthW(hwnd)
            if n == 0:
                return True
            buf = ctypes.create_unicode_buffer(n + 1)
            user32.GetWindowTextW(hwnd, buf, n + 1)
            title = buf.value
            if title in OWN_TITLES:
                return True
            rect = client_rect(hwnd)
            if rect and rect[2] >= 300 and rect[3] >= 200:
                result.append((hwnd, title, rect))
            return True

        user32.EnumWindows(_EnumProc(cb), 0)
        return result

    def find_game_window(keyword):
        kw = keyword.lower()
        cands = [w for w in list_windows() if kw in w[1].lower()]
        if not cands:
            return None
        return max(cands, key=lambda w: w[2][2] * w[2][3])[0]  # 取面积最大的
else:
    def set_dpi_aware():
        pass

    def client_rect(hwnd):
        return None

    def list_windows():
        return []

    def find_game_window(keyword):
        return None


class Capturer:
    def __init__(self, cfg):
        import mss
        self.cfg = cfg
        self.sct = mss.mss()
        self.hwnd = None
        self._last_search = 0.0

    def target_rect(self):
        """游戏画面在屏幕上的区域 (x, y, w, h)，找不到返回 None。"""
        if self.cfg["mode"] == "window":
            if not IS_WIN:
                return None
            if self.hwnd is None or not user32.IsWindow(self.hwnd):
                now = time.time()
                if now - self._last_search < 1.0:  # 每秒最多找一次窗口
                    return None
                self._last_search = now
                self.hwnd = find_game_window(self.cfg["window_keyword"])
                if self.hwnd is None:
                    return None
            return client_rect(self.hwnd)
        mons = self.sct.monitors
        idx = min(max(1, int(self.cfg.get("monitor", 1))), len(mons) - 1)
        m = mons[idx]
        return (m["left"], m["top"], m["width"], m["height"])

    def grab(self, x, y, w, h):
        try:
            shot = self.sct.grab({"left": int(x), "top": int(y), "width": int(w), "height": int(h)})
        except Exception:
            return None  # 窗口部分移出屏幕等情况
        return np.ascontiguousarray(np.asarray(shot)[:, :, :3])  # BGRA → BGR


# ════════════════════════════════════════════════════════════
#  图像工具
# ════════════════════════════════════════════════════════════

def imread_unicode(path):
    """cv2.imread 在 Windows 上不支持中文路径，用 imdecode 绕过。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"无法读取图片：{path}")
    return img


def imwrite_unicode(path, img):
    ok, buf = cv2.imencode(Path(path).suffix or ".png", img)
    if ok:
        buf.tofile(str(path))


# ════════════════════════════════════════════════════════════
#  匹配器（对应原版 MapMatcher.kt）
# ════════════════════════════════════════════════════════════

class MatchResult:
    __slots__ = ("x", "y", "conf", "heading", "scale", "inliers", "good")

    def __init__(self, x, y, conf, heading, scale=1.0, inliers=0, good=0):
        self.x, self.y, self.conf, self.heading = x, y, conf, heading
        self.scale, self.inliers, self.good = scale, inliers, good

    def copy(self, **kw):
        m = MatchResult(self.x, self.y, self.conf, self.heading, self.scale, self.inliers, self.good)
        for k, v in kw.items():
            setattr(m, k, v)
        return m


def _make_orb(cfg, nfeatures):
    return cv2.ORB_create(
        nfeatures=int(nfeatures), scaleFactor=1.2, nlevels=8, edgeThreshold=31,
        firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31,
        fastThreshold=int(cfg["fast_threshold"]),
    )


class MapMatcher:
    def __init__(self, cfg):
        self.cfg = cfg
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.orb_mini = _make_orb(cfg, cfg["mini_features"])
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.map_bgr = None
        self.map_pts = None
        self.map_desc = None
        self._mask_cache = {}

    # ---------- 大地图 ----------
    def load_map(self, path):
        t0 = time.time()
        self.map_bgr = imread_unicode(path)
        self.map_h, self.map_w = self.map_bgr.shape[:2]
        print(f"  大地图尺寸：{self.map_w} x {self.map_h}")

        st = os.stat(path)
        key = f"{Path(path).resolve()}|{st.st_size}|{st.st_mtime}|{self.cfg['map_features']}|" \
              f"{self.cfg['fast_threshold']}|{CACHE_VERSION}"
        cache = CACHE_DIR / (hashlib.sha1(key.encode("utf-8")).hexdigest()[:16] + ".npz")
        if cache.exists():
            try:
                d = np.load(cache)
                self.map_pts, self.map_desc = d["pts"], d["desc"]
                print(f"  已读取特征缓存：{len(self.map_pts)} 个特征点（{time.time() - t0:.1f}s）")
                return
            except Exception:
                pass

        print("  首次加载，正在提取大地图特征（大图可能要几十秒，之后会缓存）…")
        gray = self.clahe.apply(cv2.cvtColor(self.map_bgr, cv2.COLOR_BGR2GRAY))
        self.map_pts, self.map_desc = self._detect_tiled(gray)
        CACHE_DIR.mkdir(exist_ok=True)
        np.savez(cache, pts=self.map_pts, desc=self.map_desc)
        print(f"  特征提取完成：{len(self.map_pts)} 个特征点（{time.time() - t0:.1f}s）")

    def _detect_tiled(self, gray):
        """分块检测，保证特征点均匀分布在整张地图上。"""
        H, W = gray.shape
        tile, pad = 512, 48
        nx, ny = math.ceil(W / tile), math.ceil(H / tile)
        per = max(40, int(self.cfg["map_features"]) // (nx * ny))
        det = _make_orb(self.cfg, per)
        kps = []
        for ty in range(ny):
            for tx in range(nx):
                x0, y0 = tx * tile, ty * tile
                x1, y1 = min(W, x0 + tile), min(H, y0 + tile)
                ax0, ay0 = max(0, x0 - pad), max(0, y0 - pad)
                ax1, ay1 = min(W, x1 + pad), min(H, y1 + pad)
                for k in det.detect(gray[ay0:ay1, ax0:ax1], None):
                    gx, gy = k.pt[0] + ax0, k.pt[1] + ay0
                    if x0 <= gx < x1 and y0 <= gy < y1:
                        k.pt = (gx, gy)
                        kps.append(k)
        if not kps:
            raise RuntimeError("大地图上提取不到特征点，请换一张更清晰的地图")
        comp = _make_orb(self.cfg, len(kps))
        kps, desc = comp.compute(gray, kps)
        if desc is None or len(kps) == 0:
            raise RuntimeError("大地图特征描述计算失败")
        pts = np.float32([k.pt for k in kps])
        return pts, desc

    # ---------- 小地图 ----------
    def _mask(self, w, h):
        key = (w, h, self.cfg["minimap_circle"], self.cfg["center_mask_ratio"])
        if key not in self._mask_cache:
            m = np.full((h, w), 255, np.uint8)
            if self.cfg["minimap_circle"]:
                m[:] = 0
                cv2.ellipse(m, (w // 2, h // 2), (int(w * 0.46), int(h * 0.46)), 0, 0, 360, 255, -1)
            r = int(min(w, h) * float(self.cfg["center_mask_ratio"]))
            if r > 0:
                cv2.circle(m, (w // 2, h // 2), r, 0, -1)
            self._mask_cache[key] = m
        return self._mask_cache[key]

    def match(self, mini_bgr, hint=None):
        """hint = (x, y, radius)：只在大地图这个范围内搜索。返回 MatchResult 或 None。"""
        cfg = self.cfg
        h, w = mini_bgr.shape[:2]
        gray = self.clahe.apply(cv2.cvtColor(mini_bgr, cv2.COLOR_BGR2GRAY))
        kps, desc = self.orb_mini.detectAndCompute(gray, self._mask(w, h))
        if desc is None or len(kps) < cfg["min_matches"]:
            return None

        idx = None
        train = self.map_desc
        if hint is not None:
            hx, hy, r = hint
            d2 = (self.map_pts[:, 0] - hx) ** 2 + (self.map_pts[:, 1] - hy) ** 2
            sel = np.nonzero(d2 < r * r)[0]
            if len(sel) >= 50:
                idx = sel
                train = self.map_desc[sel]

        knn = self.bf.knnMatch(desc, train, k=2)
        good = [p[0] for p in knn if len(p) == 2 and p[0].distance < cfg["ratio_thresh"] * p[1].distance]
        if len(good) < cfg["min_matches"]:
            return None

        src = np.float32([kps[m.queryIdx].pt for m in good])
        tr = np.array([m.trainIdx for m in good])
        if idx is not None:
            tr = idx[tr]
        dst = self.map_pts[tr]

        M, inl = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=float(cfg["ransac_thresh"]),
            maxIters=3000, confidence=0.995)
        if M is None or inl is None:
            return None
        inliers = int(inl.sum())
        if inliers < cfg["min_inliers"]:
            return None
        scale = math.hypot(M[0, 0], M[1, 0])
        if not (cfg["scale_min"] <= scale <= cfg["scale_max"]):
            return None

        cx, cy = w / 2.0, h / 2.0
        mx, my = M @ np.array([cx, cy, 1.0])
        if not (-50 <= mx <= self.map_w + 50 and -50 <= my <= self.map_h + 50):
            return None
        ux, uy = M @ np.array([cx, cy - h / 4.0, 1.0])
        heading = math.degrees(math.atan2(ux - mx, -(uy - my)))  # 0=朝上(北)，顺时针

        ratio = inliers / len(good)
        conf = ratio * min(1.0, inliers / 15.0)  # 内点比例，少量内点时打折
        return MatchResult(mx, my, conf, heading, scale, inliers, len(good))


# ════════════════════════════════════════════════════════════
#  朝向箭头识别
#  小地图上北下南固定时，地图不会转，角色朝向只体现在中心的玩家箭头上。
#  校准（按 A，点一下箭头）：按颜色抠出箭头 → 找对称轴定出它当时的朝向
#                         → 把箭头存成模板。
#  运行时：把模板转 0~360°，和小地图中心逐一比对，最像的角度就是朝向。
#         只比较箭头自身的像素，所以箭头下面的道路、水面不会干扰。
# ════════════════════════════════════════════════════════════

_KERNEL3 = np.ones((3, 3), np.uint8)


def _lab(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)


def ang_diff(a, b):
    """a - b，结果落在 [-180, 180)。"""
    return (a - b + 180.0) % 360.0 - 180.0


def mask_heading(mask):
    """箭头二值图 → (角度, 对称度)；角度 0=朝上，顺时针，单位度。
    箭头左右对称：找一条对折后重合最好的轴，就是指向的那条线；
    再看沿轴哪头伸得更远（尖头那端），确定朝哪边。"""
    ys, xs = np.nonzero(mask)
    if len(xs) < 15:
        return None
    H, W = mask.shape[:2]
    cx, cy = xs.mean(), ys.mean()
    px, py = xs - cx, ys - cy
    if len(px) > 1500:  # 点太多就抽样，够用且快
        sel = np.random.default_rng(0).choice(len(px), 1500, replace=False)
        sx, sy = px[sel], py[sel]
    else:
        sx, sy = px, py

    deg = np.arange(180.0)
    th = np.radians(deg)
    ux, uy = np.sin(th)[:, None], -np.cos(th)[:, None]
    d = ux * sx + uy * sy
    ix = np.rint(2 * d * ux - sx + cx).astype(np.int32)  # 关于轴的镜像点
    iy = np.rint(2 * d * uy - sy + cy).astype(np.int32)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    hit = np.zeros(ix.shape, bool)
    hit[ok] = mask[iy[ok], ix[ok]] > 0
    score = hit.mean(axis=1)

    k = int(np.argmax(score))
    s0, s1, s2 = score[k - 1], score[k], score[(k + 1) % 180]
    den = s0 - 2 * s1 + s2
    off = 0.5 * (s0 - s2) / den if den < 0 else 0.0  # 抛物线插值到 1° 以内
    ang = float(deg[k] + np.clip(off, -0.5, 0.5))

    r = math.radians(ang)
    proj = px * math.sin(r) - py * math.cos(r)
    if -np.percentile(proj, 1) > np.percentile(proj, 99):
        ang += 180.0
    return ang % 360.0, float(s1)


def _color_mask(patch_bgr, color, tol, lab=None):
    lab = _lab(patch_bgr) if lab is None else lab
    dist = np.linalg.norm(lab - np.float32(color), axis=2)
    return cv2.morphologyEx((dist < tol).astype(np.uint8), cv2.MORPH_OPEN, _KERNEL3)


def _pick_component(mask, pt):
    """取包含 pt 的连通块；pt 正好不在块上时取离它最近的。返回 (mask, 是否碰到边缘)。"""
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None, False
    H, W = mask.shape[:2]
    x, y = int(pt[0]), int(pt[1])
    k = labels[y, x] if 0 <= y < H and 0 <= x < W else 0
    if k == 0:
        d = np.hypot(cents[1:, 0] - pt[0], cents[1:, 1] - pt[1])
        k = 1 + int(np.argmin(d))
        if d[k - 1] > 0.08 * max(H, W):
            return None, False
    bx, by, bw, bh, area = stats[k]
    edge = bx == 0 or by == 0 or bx + bw >= W or by + bh >= H or area > 0.3 * H * W
    return (labels == k).astype(np.uint8), edge


def segment_arrow(patch, seed, color=None, tol=None):
    """校准用：从点击位置 seed 抠出箭头。color/tol 为空时自动估计。
    返回 {mask, color, tol, ang, sym} 或 None。"""
    lab = _lab(patch)
    sx, sy = int(seed[0]), int(seed[1])
    min_area = 0.0008 * patch.shape[0] * patch.shape[1]

    def evaluate(col, t):
        comp, edge = _pick_component(_color_mask(patch, col, t, lab), (sx, sy))
        if comp is None or edge:
            return None
        res = mask_heading(comp)
        if res is None:
            return None
        return {"mask": comp, "color": [float(c) for c in col], "tol": float(t),
                "ang": res[0], "sym": res[1], "area": int(comp.sum())}

    if color is not None and tol is not None:
        return evaluate(color, tol)

    def tune(col):
        # 颜色范围从小往大试：太小只抠到一部分（不对称），太大会把描边、背景也粘进来。
        # 把面积基本不变的连续几档算作一个「平台」，在对称度接近最高的平台里取最早（最贴身）的，
        # 再取平台中间那档，两边都留余量
        cands = []
        for t in range(6, 101, 3):
            r = evaluate(col, t)
            if r is not None and r["area"] >= min_area:
                cands.append(r)
        if not cands:
            return None
        groups = []
        for r in cands:
            g = groups[-1] if groups else None
            if g and abs(r["area"] - g[0]["area"]) <= 0.08 * g[0]["area"] \
                    and abs(ang_diff(r["ang"], g[0]["ang"])) < 10:
                g.append(r)
            else:
                groups.append([r])
        if any(len(g) >= 2 for g in groups):
            groups = [g for g in groups if len(g) >= 2]
        score = [float(np.median([r["sym"] for r in g])) for g in groups]
        top = int(np.argmax(score))
        for g, sc in zip(groups, score):
            if sc >= score[top] - 0.02 and g[0]["area"] >= 0.35 * groups[top][0]["area"]:
                return g[len(g) // 2]
        return groups[top][len(groups[top]) // 2]

    color = np.median(lab[max(0, sy - 1):sy + 2, max(0, sx - 1):sx + 2].reshape(-1, 3), axis=0)
    res = tune(color)
    if res is None:
        return None
    # 用整个箭头的颜色中位数再估一次（适应渐变 / 两种颜色拼成的箭头）
    res2 = tune(np.median(lab[res["mask"] > 0], axis=0))
    if res2 is not None and res2["sym"] >= res["sym"]:
        res = res2
    return res


def build_arrow_template(raw, mask_up):
    """校准用：把放大图上的箭头 mask 缩回原尺寸，连同一圈描边存成 BGRA 模板，
    返回 (模板 PNG 的 base64, 箭头中心在 raw 上的坐标)。"""
    import base64
    h, w = raw.shape[:2]
    m = cv2.resize(mask_up.astype(np.float32), (w, h), interpolation=cv2.INTER_AREA) > 0.4
    m = cv2.dilate(m.astype(np.uint8), _KERNEL3)  # 多带一圈，把箭头的描边也算进去
    ys, xs = np.nonzero(m)
    if len(xs) < 6:
        return None, None
    cx, cy = int(round(xs.mean())), int(round(ys.mean()))
    half = int(math.ceil(np.hypot(xs - cx, ys - cy).max())) + 1
    pad = half + 2
    big = cv2.copyMakeBorder(raw, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    bm = cv2.copyMakeBorder(m, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=0)
    sl = (slice(cy + pad - half, cy + pad + half + 1), slice(cx + pad - half, cx + pad + half + 1))
    bgra = np.dstack([big[sl], bm[sl] * 255]).astype(np.uint8)
    ok, buf = cv2.imencode(".png", bgra)
    return (base64.b64encode(buf.tobytes()).decode("ascii") if ok else None), (cx, cy)


def arrow_config(mini_shape, x0, y0, raw, res, up, flip):
    """校准结果 → 写进配置的 arrow 字段（位置按小地图比例记，换分辨率也能用）。"""
    h, w = mini_shape[:2]
    tpl, c = build_arrow_template(raw, res["mask"])
    if tpl is None:
        return None
    return {"cx": round((x0 + c[0]) / w, 4), "cy": round((y0 + c[1]) / h, 4), "s": min(w, h),
            "a0": round((res["ang"] + (180.0 if flip else 0.0)) % 360.0, 2), "flip": bool(flip),
            "color": [round(v, 1) for v in res["color"]], "tol": round(res["tol"], 1), "tpl": tpl}


class ArrowDetector:
    """运行时：在小地图上找玩家箭头的朝向（度，0=上，顺时针）；找不到返回 None。"""

    STEP = 2  # 模板每 2° 转一份，最后插值到 1° 以内

    def __init__(self, cfg):
        self.cfg = cfg
        self._key = None
        self._bank = None
        self.last_rmse = None

    def _build(self, s):
        import base64
        a = self.cfg["arrow"]
        key = (hash(a["tpl"]), s)
        if key == self._key:
            return self._bank
        self._key, self._bank = key, None
        try:
            buf = np.frombuffer(base64.b64decode(a["tpl"]), np.uint8)
            bgra = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
        except Exception:
            return None
        if bgra is None or bgra.ndim != 3 or bgra.shape[2] != 4:
            return None
        k = s / float(a["s"])                          # 换了分辨率：模板跟着缩放
        side0 = bgra.shape[0] * k
        f = min(1.0, 40.0 / max(1.0, side0 * 1.42))   # 箭头很大时降采样，控制计算量
        size = max(5, int(round(side0 * f)) | 1)
        T = cv2.resize(bgra[:, :, :3], (size, size), interpolation=cv2.INTER_AREA).astype(np.float32)
        M = cv2.resize(bgra[:, :, 3], (size, size), interpolation=cv2.INTER_AREA).astype(np.float32) / 255
        D = int(math.ceil(size * 1.42)) | 1           # 留出旋转后的空间
        p = (D - size) // 2
        T = cv2.copyMakeBorder(T, p, D - size - p, p, D - size - p, cv2.BORDER_CONSTANT, value=0)
        M = cv2.copyMakeBorder(M, p, D - size - p, p, D - size - p, cv2.BORDER_CONSTANT, value=0)
        c = (D - 1) / 2.0
        Ts, Ms = [], []
        for deg in range(0, 360, self.STEP):
            R = cv2.getRotationMatrix2D((c, c), -deg, 1.0)  # 负角度 = 顺时针
            Ts.append(cv2.warpAffine(T, R, (D, D), flags=cv2.INTER_LINEAR))
            Ms.append(cv2.warpAffine(M, R, (D, D), flags=cv2.INTER_LINEAR) > 0.5)
        M3 = np.repeat(np.float32(Ms)[..., None], 3, axis=3).reshape(len(Ms), -1)
        MT = M3 * np.float32(Ts).reshape(len(Ts), -1)
        A = (MT * np.float32(Ts).reshape(len(Ts), -1)).sum(1)
        cnt = np.maximum(M3.sum(1), 1.0)
        mg = max(2, int(round(D * 0.12)))             # 允许箭头中心有几个像素的偏移
        self._bank = (f, D, mg, M3, MT, A, cnt)
        return self._bank

    def detect(self, mini):
        a = self.cfg.get("arrow")
        if not a or not a.get("tpl"):
            return None
        h, w = mini.shape[:2]
        bank = self._build(min(w, h))
        if bank is None:
            return None
        f, D, mg, M3, MT, A, cnt = bank

        # 以校准时的箭头中心为圆心取一块，缩放到模板的尺度
        Wn = D + 2 * mg
        half = Wn / (2.0 * f)
        cx, cy = a["cx"] * w, a["cy"] * h
        x0, y0 = int(round(cx - half)), int(round(cy - half))
        x1, y1 = int(round(cx + half)), int(round(cy + half))
        pl, pt, pr, pb = max(0, -x0), max(0, -y0), max(0, x1 - w), max(0, y1 - h)
        crop = mini[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
        if crop.size == 0:
            return None
        if pl or pt or pr or pb:
            crop = cv2.copyMakeBorder(crop, pt, pb, pl, pr, cv2.BORDER_REPLICATE)
        win = cv2.resize(crop, (Wn, Wn), interpolation=cv2.INTER_AREA if f < 1 else cv2.INTER_LINEAR)
        win = win.astype(np.float32)

        # 所有角度 × 所有偏移 的加权平方差，一次矩阵乘法算完
        I = np.lib.stride_tricks.sliding_window_view(win, (D, D, 3)).reshape(-1, D * D * 3)
        ssd = A[:, None] - 2.0 * (MT @ I.T) + M3 @ (I * I).T
        e = ssd.min(axis=1) / cnt
        k = int(np.argmin(e))
        n = len(e)
        e0, e1, e2 = e[k - 1], e[k], e[(k + 1) % n]
        den = e0 - 2 * e1 + e2
        off = 0.5 * (e0 - e2) / den if den > 0 else 0.0

        self.last_rmse = rmse = math.sqrt(max(0.0, float(e1)))
        if rmse > float(self.cfg["arrow_max_rmse"]):
            return None  # 哪个角度都不像：箭头被挡住了 / UI 隐藏了
        if e1 > 0.6 * float(np.median(e)):
            return None  # 各角度差不多：没认出来
        return (float(a["a0"]) + (k + float(np.clip(off, -0.5, 0.5))) * self.STEP) % 360.0


# ════════════════════════════════════════════════════════════
#  跟踪状态：平滑 + 惯性 + 防跳变（对应原版 applySmoothing）
# ════════════════════════════════════════════════════════════

class Tracker:
    def __init__(self, cfg):
        self.cfg = cfg
        self.last = None
        self.fails = 0
        self.pending = None
        self.scale = None
        self.last_raw = None
        self.arrow = None        # 玩家箭头在小地图上的朝向（度，平滑后）
        self.arrow_fails = 0
        self.arrow_pending = None

    def hint(self, mini_shape):
        if self.last is None or self.scale is None or self.fails >= 5:
            return None
        h, w = mini_shape[:2]
        r = max(w, h) * self.scale * 1.5 + 150
        return (self.last.x, self.last.y, r)

    def update(self, m):
        cfg = self.cfg
        self.last_raw = m
        if m is None or m.conf < cfg["conf_threshold"]:
            self.fails += 1
            if self.last is not None:
                self.last = self.last.copy(conf=self.last.conf * 0.5)  # 保持位置、降低置信度
            return

        last = self.last
        if last is None or last.conf < 0.2:
            self._accept(m)
            return

        dist = math.hypot(m.x - last.x, m.y - last.y)
        if dist > cfg["jump_dist"]:
            # 跳变：高置信度直接接受，否则需要下一帧在附近再次确认（比如传送）
            if m.conf >= 0.7 or (self.pending is not None and
                                 math.hypot(m.x - self.pending.x, m.y - self.pending.y) < cfg["jump_dist"] / 5):
                self._accept(m)
            else:
                self.pending = m
                self.fails += 1
            return

        a = cfg["smooth_alpha"]
        self._accept(m.copy(x=last.x * (1 - a) + m.x * a, y=last.y * (1 - a) + m.y * a,
                            heading=(last.heading + a * ang_diff(m.heading, last.heading)) % 360.0))

    def _accept(self, m):
        self.last = m
        self.fails = 0
        self.pending = None
        self.scale = m.scale if self.scale is None else self.scale * 0.8 + m.scale * 0.2

    @property
    def lost(self):
        return self.fails >= 10

    def update_arrow(self, ang):
        if ang is None:
            self.arrow_fails += 1
            return
        self.arrow_fails = 0
        if self.arrow is None:
            self.arrow = ang
        elif abs(ang_diff(ang, self.arrow)) > 75:
            # 突然转了很大角度：下一帧还是这个方向才采用，防止偶尔认错一帧乱甩
            if self.arrow_pending is not None and abs(ang_diff(ang, self.arrow_pending)) < 25:
                self.arrow, self.arrow_pending = ang, None
            else:
                self.arrow_pending = ang
        else:
            self.arrow_pending = None
            a = float(self.cfg["arrow_smooth"])
            self.arrow = (self.arrow + a * ang_diff(ang, self.arrow)) % 360.0

    def heading(self):
        """悬浮窗箭头方向 = 小地图相对大地图的旋转 + 玩家箭头在小地图上的朝向。
        小地图上北下南固定时前一项≈0；没校准玩家箭头时只有前一项（就是一直朝上）。"""
        rot = self.last.heading if self.last is not None else 0.0
        return rot % 360.0 if self.arrow is None else (rot + self.arrow) % 360.0


# ════════════════════════════════════════════════════════════
#  显示
# ════════════════════════════════════════════════════════════

RES_COLORS = {  # BGR，对应原版 ResourceType 颜色
    "NPC": (0, 152, 255), "MONSTER": (54, 67, 244), "COLLECTION": (80, 175, 76),
    "PORTAL": (243, 150, 33), "DUNGEON": (176, 39, 156), "SHOP": (7, 193, 255),
    "QUEST": (212, 188, 0), "LANDMARK": (139, 125, 96),
}


def load_resources():
    if not RESOURCES_PATH.exists():
        return []
    try:
        data = json.loads(RESOURCES_PATH.read_text(encoding="utf-8"))
        items = data["resources"] if isinstance(data, dict) else data
        return [(float(r["x"]), float(r["y"]), str(r.get("name", "")), str(r.get("type", "LANDMARK")).upper())
                for r in items]
    except Exception as e:
        print(f"⚠ resources.json 读取失败：{e}")
        return []


class TextDrawer:
    """OpenCV 不能写中文，用 Pillow + 微软雅黑。"""

    def __init__(self):
        self.ok = False
        try:
            from PIL import Image, ImageDraw, ImageFont
            self.Image, self.ImageDraw = Image, ImageDraw
            for f in ("C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
                      "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
                      "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
                if os.path.exists(f):
                    self.fonts = {s: ImageFont.truetype(f, s) for s in (12, 14, 16)}
                    self.ok = True
                    break
        except ImportError:
            pass

    def draw(self, img, items):
        """items: [(text, (x, y), (b, g, r), size)]"""
        if not items:
            return img
        if not self.ok:
            for text, (x, y), color, size in items:
                cv2.putText(img, text.encode("ascii", "replace").decode(), (x, y + size),
                            cv2.FONT_HERSHEY_SIMPLEX, size / 30, color, 1, cv2.LINE_AA)
            return img
        pil = self.Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = self.ImageDraw.Draw(pil)
        for text, (x, y), (b, g, r), size in items:
            d.text((x, y), text, font=self.fonts.get(size, self.fonts[14]), fill=(r, g, b))
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def conf_color(conf, thresh):
    if conf >= 0.6:
        return (80, 200, 80)    # 绿
    if conf >= thresh:
        return (0, 210, 255)    # 黄
    return (60, 60, 230)        # 红


class Renderer:
    def __init__(self, map_bgr, cfg, resources):
        self.map = map_bgr
        self.cfg = cfg
        self.res = resources
        self.text = TextDrawer()
        H, W = map_bgr.shape[:2]
        self.ov_scale = min(1.0, 640 / max(W, H))
        self.overview = cv2.resize(map_bgr, (max(1, int(W * self.ov_scale)), max(1, int(H * self.ov_scale))),
                                   interpolation=cv2.INTER_AREA)
        self.expanded = False

    def _crop(self, cx, cy, half):
        H, W = self.map.shape[:2]
        size = 2 * half
        out = np.full((size, size, 3), 30, np.uint8)
        x0, y0 = int(round(cx)) - half, int(round(cy)) - half
        sx0, sy0 = max(0, x0), max(0, y0)
        sx1, sy1 = min(W, x0 + size), min(H, y0 + size)
        if sx1 > sx0 and sy1 > sy0:
            out[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = self.map[sy0:sy1, sx0:sx1]
        return out, x0, y0

    def _marker(self, img, px, py, heading, color):
        px, py = int(px), int(py)
        rad = math.radians(heading)
        tip = (int(px + 24 * math.sin(rad)), int(py - 24 * math.cos(rad)))
        cv2.arrowedLine(img, (px, py), tip, (255, 255, 255), 4, cv2.LINE_AA, tipLength=0.4)
        cv2.arrowedLine(img, (px, py), tip, color, 2, cv2.LINE_AA, tipLength=0.4)
        cv2.circle(img, (px, py), 8, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 6, color, -1, cv2.LINE_AA)

    def render(self, tracker, status, warn=""):
        cfg = self.cfg
        last = tracker.last
        texts = []
        if self.expanded:
            img = self.overview.copy()
            s = self.ov_scale
            if cfg["show_resources"]:
                for x, y, name, t in self.res:
                    cv2.circle(img, (int(x * s), int(y * s)), 3, RES_COLORS.get(t, (220, 220, 220)), -1)
            if last is not None:
                self._marker(img, last.x * s, last.y * s, tracker.heading(),
                             conf_color(last.conf, cfg["conf_threshold"]))
        else:
            view = int(cfg["view_size"])
            zoom = float(cfg["view_zoom"])
            half = max(16, int(view / (2 * zoom)))
            if last is not None:
                cx, cy = last.x, last.y
            else:
                cx, cy = self.map.shape[1] / 2, self.map.shape[0] / 2
            crop, x0, y0 = self._crop(cx, cy, half)
            img = cv2.resize(crop, (view, view), interpolation=cv2.INTER_LINEAR if zoom > 1 else cv2.INTER_AREA)
            k = view / (2 * half)
            if cfg["show_resources"]:
                for x, y, name, t in self.res:
                    px, py = (x - x0) * k, (y - y0) * k
                    if 0 <= px < view and 0 <= py < view:
                        cv2.circle(img, (int(px), int(py)), 5, RES_COLORS.get(t, (220, 220, 220)), -1, cv2.LINE_AA)
                        texts.append((name, (int(px) + 7, int(py) - 8), (255, 255, 255), 12))
            if last is not None:
                self._marker(img, (last.x - x0) * k, (last.y - y0) * k, tracker.heading(),
                             conf_color(last.conf, cfg["conf_threshold"]))

        h, w = img.shape[:2]
        bar_h = 44 if warn else 26
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)
        if last is not None and not status:
            c = conf_color(last.conf, cfg["conf_threshold"])
            state = "丢失" if tracker.lost else "定位中"
            texts.append((f"{state}  ({last.x:.0f}, {last.y:.0f})  置信度 {last.conf:.2f}", (6, 4), c, 14))
        else:
            texts.append((status or "等待匹配…", (6, 4), (230, 230, 230), 14))
        if warn:
            texts.append((warn, (6, 24), (60, 160, 255), 12))
        return self.text.draw(img, texts)


# ════════════════════════════════════════════════════════════
#  校准
# ════════════════════════════════════════════════════════════

def calibrate(capturer, cfg):
    print("\n【校准小地图】3 秒后截取游戏画面，请现在切换到游戏…")
    for i in (3, 2, 1):
        print(f"  {i}…")
        time.sleep(1)
    rect = capturer.target_rect()
    if rect is None:
        print("  ✗ 找不到游戏画面（游戏没开、最小化了，或窗口关键词不对）")
        return False
    gx, gy, gw, gh = rect
    shot = capturer.grab(gx, gy, gw, gh)
    if shot is None:
        print("  ✗ 截图失败：游戏窗口可能有一部分在屏幕外")
        return False
    s = min(1.0, 1400 / gw, 820 / gh)
    small = cv2.resize(shot, (int(gw * s), int(gh * s)), interpolation=cv2.INTER_AREA) if s < 1 else shot

    print("  请在弹出的窗口里，用鼠标拖一个框把小地图框住（尽量贴着边），按回车确认，按 C 取消。")
    cv2.namedWindow(CALIB_WIN, cv2.WINDOW_AUTOSIZE)
    try:
        cv2.setWindowProperty(CALIB_WIN, cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass
    r = cv2.selectROI(CALIB_WIN, small, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(CALIB_WIN)
    cv2.waitKey(1)
    if r[2] < 10 or r[3] < 10:
        print("  已取消校准")
        return False

    x, y, w, h = [int(round(v / s)) for v in r]
    cfg["profiles"][size_key(gw, gh)] = [x, y, w, h]
    save_config(cfg)
    CACHE_DIR.mkdir(exist_ok=True)
    imwrite_unicode(CACHE_DIR / "minimap_preview.png", shot[y:y + h, x:x + w])
    print(f"  ✓ 已保存：画面 {gw}x{gh}，小地图区域 x={x} y={y} w={w} h={h}")
    print(f"    裁剪预览图：{CACHE_DIR / 'minimap_preview.png'}（可打开检查是否框准）")

    print("\n【校准朝向箭头】在弹出的放大图里，点一下小地图中间的玩家箭头（不需要可按 Esc 跳过）")
    calibrate_arrow(shot[y:y + h, x:x + w], cfg)
    return True


def _arrow_canvas(patch, st, text):
    """朝向校准窗口的画面：上面 3 行说明，下面是放大的小地图中心。"""
    ph, pw = patch.shape[:2]
    bar = 70
    canvas = np.full((bar + ph, max(pw, 560), 3), 32, np.uint8)
    view = patch.copy()
    res = st["res"]
    if res is not None:
        cs, _ = cv2.findContours(res["mask"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(view, cs, -1, (0, 255, 0), 2, cv2.LINE_AA)
        ys, xs = np.nonzero(res["mask"])
        cx, cy = xs.mean(), ys.mean()
        L = max(40.0, 1.8 * float(np.hypot(xs - cx, ys - cy).max()))
        rad = math.radians(res["ang"] + (180.0 if st["flip"] else 0.0))
        p0, p1 = (int(cx), int(cy)), (int(cx + L * math.sin(rad)), int(cy - L * math.cos(rad)))
        cv2.arrowedLine(view, p0, p1, (255, 255, 255), 6, cv2.LINE_AA, tipLength=0.25)
        cv2.arrowedLine(view, p0, p1, (40, 40, 255), 3, cv2.LINE_AA, tipLength=0.25)
    if st["seed"] is not None:
        cv2.drawMarker(view, (int(st["seed"][0]), int(st["seed"][1])), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
    canvas[bar:, :pw] = view

    lines = [("点一下小地图中间玩家箭头的身体（有颜色的部分）", (8, 4), (235, 235, 235), 14)]
    if res is not None:
        deg = (res["ang"] + (180.0 if st["flip"] else 0.0)) % 360.0
        lines.append((f"绿线 = 识别到的箭头，红箭头 = 识别的朝向（{deg:.0f}°，对称度 {res['sym']:.2f}）",
                      (8, 26), (120, 230, 120), 12))
    elif st["seed"] is not None:
        lines.append(("这里没识别出箭头：换个位置点（箭头中间颜色最纯的地方），或按 +/- 调范围",
                      (8, 26), (90, 160, 255), 12))
    lines.append(("红箭头和游戏里反了按 F · 绿线没框全/溢出按 +/- · 回车确认 · Esc 跳过",
                  (8, 46), (200, 200, 200), 12))
    return text.draw(canvas, lines)


def calibrate_arrow(mini, cfg):
    """让用户点一下小地图上的玩家箭头，记下它的样子，之后就能识别朝向。"""
    h, w = mini.shape[:2]
    s = min(w, h)
    R = max(8, int(s * 0.25))
    cx, cy = w // 2, h // 2
    x0, y0 = max(0, cx - R), max(0, cy - R)
    raw = mini[y0:min(h, cy + R), x0:min(w, cx + R)].copy()
    up = float(np.clip(480.0 / max(raw.shape[:2]), 1.0, 8.0))
    patch = cv2.resize(raw, None, fx=up, fy=up, interpolation=cv2.INTER_CUBIC)
    ph, pw = patch.shape[:2]
    bar = 70
    text = TextDrawer()

    old = cfg.get("arrow") or {}
    st = {"seed": None, "color": None, "tol": None, "flip": bool(old.get("flip")), "res": None, "dirty": False}
    if old.get("color") and old.get("tol"):  # 校准过：直接用上次的结果预览，没问题按回车就行
        sx, sy = (old["cx"] * w - x0) * up, (old["cy"] * h - y0) * up
        if 0 <= sx < pw and 0 <= sy < ph:
            st.update(seed=(sx, sy), color=old["color"], tol=old["tol"], dirty=True)

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and bar <= y < bar + ph and x < pw:
            st.update(seed=(x, y - bar), color=None, tol=None, flip=False, dirty=True)

    cv2.namedWindow(ARROW_WIN, cv2.WINDOW_AUTOSIZE)
    try:
        cv2.setWindowProperty(ARROW_WIN, cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass
    cv2.setMouseCallback(ARROW_WIN, on_mouse)

    ok = False
    canvas = None
    while True:
        if st["dirty"]:
            st["dirty"] = False
            st["res"] = segment_arrow(patch, st["seed"], st["color"], st["tol"])
            if st["res"] is not None:
                st["color"], st["tol"] = st["res"]["color"], st["res"]["tol"]
        canvas = _arrow_canvas(patch, st, text)
        cv2.imshow(ARROW_WIN, canvas)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10):
            if st["res"] is not None:
                ok = True
                break
        elif key == 27:
            break
        elif key in (ord("f"), ord("F")) and st["res"] is not None:
            st["flip"] = not st["flip"]
        elif key in (ord("+"), ord("="), ord("-"), ord("_")) and st["seed"] is not None and st["tol"]:
            st["tol"] = float(np.clip(st["tol"] * (1.15 if key in (ord("+"), ord("=")) else 1 / 1.15), 3, 160))
            st["dirty"] = True
        try:
            if cv2.getWindowProperty(ARROW_WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            break
    try:
        cv2.destroyWindow(ARROW_WIN)
    except cv2.error:
        pass
    cv2.waitKey(1)

    if not ok:
        print("  已跳过朝向校准" + ("（沿用上次的结果）" if cfg.get("arrow") else "（悬浮窗箭头会一直朝上，之后可按 A 校准）"))
        return False
    arrow = arrow_config(mini.shape, x0, y0, raw, st["res"], up, st["flip"])
    if arrow is None:
        print("  ✗ 箭头太小，没法做模板，请重试")
        return False
    cfg["arrow"] = arrow
    save_config(cfg)
    CACHE_DIR.mkdir(exist_ok=True)
    imwrite_unicode(CACHE_DIR / "arrow_preview.png", canvas)
    print(f"  ✓ 朝向箭头已校准（当前朝向 {arrow['a0']:.0f}°），预览图：{CACHE_DIR / 'arrow_preview.png'}")
    return True


# ════════════════════════════════════════════════════════════
#  设置向导
# ════════════════════════════════════════════════════════════

def ask(prompt, default=None):
    tip = f"（直接回车 = {default}）" if default not in (None, "") else ""
    v = input(f"{prompt}{tip}：").strip().strip('"')
    return v if v else (default if default is not None else "")


def pick_map_file(current):
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="选择游戏大地图图片",
            filetypes=[("图片", "*.png *.jpg *.jpeg *.bmp *.webp"), ("所有文件", "*.*")])
        root.destroy()
        if path:
            return path
    except Exception:
        pass
    return ask("请输入大地图图片的完整路径", current)


def setup(cfg):
    print("=" * 56)
    print("  洛克王国世界 · 小地图定位  设置向导")
    print("=" * 56)

    print("\n【1/3】选择大地图图片（会弹出文件选择框）")
    while True:
        path = pick_map_file(cfg.get("map_path", ""))
        if path and Path(path).exists():
            try:
                img = imread_unicode(path)
                print(f"  ✓ {path}（{img.shape[1]}x{img.shape[0]}）")
                cfg["map_path"] = str(Path(path).resolve())
                break
            except Exception as e:
                print(f"  ✗ {e}")
        else:
            print("  ✗ 文件不存在，请重新选择")

    print("\n【2/3】怎么找到游戏画面？")
    print("  1) 跟随游戏窗口（推荐：全屏/窗口化都适用，窗口移动、切换分辨率都能跟上）")
    print("  2) 截取整个显示器（方式 1 找不到窗口时再用）")
    mode = ask("请输入 1 或 2", "1")
    if mode == "2":
        import mss
        mons = mss.mss().monitors[1:]
        for i, m in enumerate(mons, 1):
            print(f"  显示器 {i}：{m['width']}x{m['height']}  位置({m['left']},{m['top']})")
        cfg["mode"] = "monitor"
        cfg["monitor"] = int(ask("游戏在哪个显示器", "1") or 1)
    else:
        cfg["mode"] = "window"
        print("  请先把游戏打开（全屏或窗口都行）。当前可见窗口：")
        wins = sorted(list_windows(), key=lambda w: -w[2][2] * w[2][3])[:20]
        for i, (_, title, rc) in enumerate(wins, 1):
            print(f"   {i:2d}. {title[:50]:<50}  {rc[2]}x{rc[3]}")
        v = ask("输入游戏窗口的序号；或直接输入窗口标题里的关键词", cfg["window_keyword"])
        if v.isdigit() and 1 <= int(v) <= len(wins):
            title = wins[int(v) - 1][1]
            kw = ask(f"  窗口标题是「{title}」，用哪个关键词识别它（标题会变的话取不变的部分）", title)
            cfg["window_keyword"] = kw
        else:
            cfg["window_keyword"] = v

    print("\n【3/3】小地图形状")
    cfg["minimap_circle"] = ask("小地图是圆形的吗？y=圆形 n=方形", "y").lower().startswith("y")

    save_config(cfg)
    print(f"\n✓ 设置已保存到 {CONFIG_PATH}")
    print("  接下来启动追踪；当前分辨率没校准过会自动进入校准。\n")


# ════════════════════════════════════════════════════════════
#  主循环
# ════════════════════════════════════════════════════════════

def rects_overlap(a, b):
    return not (a[0] + a[2] <= b[0] or b[0] + b[2] <= a[0] or a[1] + a[3] <= b[1] or b[1] + b[3] <= a[1])


def run(cfg, force_calibrate=False):
    print("正在加载大地图…")
    matcher = MapMatcher(cfg)
    matcher.load_map(cfg["map_path"])
    capturer = Capturer(cfg)
    tracker = Tracker(cfg)
    arrow_det = ArrowDetector(cfg)
    renderer = Renderer(matcher.map_bgr, cfg, load_resources())

    # 首次：当前分辨率没校准就先校准
    rect = capturer.target_rect()
    if rect is None and cfg["mode"] == "window":
        print(f"等待游戏窗口（关键词「{cfg['window_keyword']}」）…")
        while rect is None:
            time.sleep(1)
            rect = capturer.target_rect()
    if force_calibrate or (rect and resolve_minimap(cfg, rect[2], rect[3]) is None):
        calibrate(capturer, cfg)

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)
    try:
        cv2.setWindowProperty(WIN_NAME, cv2.WND_PROP_TOPMOST, 1)
    except Exception:
        pass

    def place_window():
        if cfg.get("window_pos"):
            cv2.moveWindow(WIN_NAME, *cfg["window_pos"])
            return
        r = capturer.target_rect()
        mm = resolve_minimap(cfg, r[2], r[3]) if r else None
        if r and mm:  # 放到小地图对侧，避免挡住小地图
            x = r[0] + 30 if mm[0] + mm[2] / 2 > r[2] / 2 else r[0] + r[2] - cfg["view_size"] - 30
            cv2.moveWindow(WIN_NAME, int(x), int(r[1] + r[3] / 2 - cfg["view_size"] / 2))
        else:
            cv2.moveWindow(WIN_NAME, 30, 200)

    place_window()

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            renderer.expanded = not renderer.expanded
        elif event == cv2.EVENT_MOUSEWHEEL:
            d = flags >> 16          # 滚轮滚动量在 flags 的高 16 位（Python 版 cv2 没有 getMouseWheelDelta）
            if d >= 0x8000:          # 有的版本 flags 是无符号数，转回负数
                d -= 0x10000
            if d == 0:
                return
            cfg["view_zoom"] = min(6.0, max(0.25, cfg["view_zoom"] * (1.2 if d > 0 else 1 / 1.2)))

    cv2.setMouseCallback(WIN_NAME, on_mouse)

    print("\n追踪已启动。悬浮窗操作（需先点一下悬浮窗）：")
    print("  双击 = 展开/收起全图   滚轮 或 +/- = 缩放   R = 资源点开关")
    print("  空格 = 暂停/继续       C = 重新校准         A = 校准朝向箭头")
    print("  Q/Esc = 退出\n")

    last_t = 0.0
    paused = False
    black_count = 0
    topmost_set = False
    while True:
        status, warn = "", ""
        now = time.time()
        rect = capturer.target_rect()
        mini_abs = None
        if rect is None:
            status = "等待游戏窗口…（游戏未打开或已最小化）" if cfg["mode"] == "window" else "找不到显示器"
        else:
            mm = resolve_minimap(cfg, rect[2], rect[3])
            if mm is not None:  # 限制在游戏画面内
                mx, my = min(max(0, mm[0]), rect[2] - 10), min(max(0, mm[1]), rect[3] - 10)
                mm = [mx, my, min(mm[2], rect[2] - mx), min(mm[3], rect[3] - my)]
            if mm is None:
                status = f"画面 {rect[2]}x{rect[3]} 还没校准，点悬浮窗后按 C"
            else:
                mini_abs = (rect[0] + mm[0], rect[1] + mm[1], mm[2], mm[3])
                if paused:
                    status = "已暂停（空格继续）"
                elif now - last_t >= cfg["interval"]:
                    last_t = now
                    mini = capturer.grab(*mini_abs)
                    if mini is None:
                        tracker.update(None)
                    else:
                        black_count = black_count + 1 if mini.mean() < 4 else 0
                        tracker.update(matcher.match(mini, tracker.hint(mini.shape)))
                        if cfg.get("arrow"):
                            tracker.update_arrow(arrow_det.detect(mini))
        if black_count >= 15:
            warn = "截图全黑：独占全屏无法截图，请把游戏改成无边框全屏或窗口化"
        elif mini_abs is not None:
            try:
                wr = cv2.getWindowImageRect(WIN_NAME)
                if wr[2] > 0 and rects_overlap(wr, mini_abs):
                    warn = "悬浮窗挡住了小地图，请拖开"
            except Exception:
                pass
        if not warn and tracker.lost and not status:
            warn = "持续丢失：检查校准是否准确、大地图是否对应当前区域"
        if not warn and not status and mini_abs is not None:
            if not cfg.get("arrow"):
                warn = "朝向未校准：点一下悬浮窗按 A，再点小地图上的玩家箭头"
            elif tracker.arrow_fails >= 30:
                warn = "认不出玩家箭头，朝向没在更新：可按 A 重新校准"

        cv2.imshow(WIN_NAME, renderer.render(tracker, status, warn))
        if not topmost_set:  # 窗口真正显示后再置顶一次更可靠
            try:
                cv2.setWindowProperty(WIN_NAME, cv2.WND_PROP_TOPMOST, 1)
            except Exception:
                pass
            topmost_set = True
        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        elif key == ord(" "):
            paused = not paused
        elif key in (ord("r"), ord("R")):
            cfg["show_resources"] = not cfg["show_resources"]
        elif key in (ord("+"), ord("=")):
            cfg["view_zoom"] = min(6.0, cfg["view_zoom"] * 1.2)
        elif key in (ord("-"), ord("_")):
            cfg["view_zoom"] = max(0.25, cfg["view_zoom"] / 1.2)
        elif key in (ord("c"), ord("C")):
            try:
                pos = cv2.getWindowImageRect(WIN_NAME)[:2]
            except Exception:
                pos = (30, 200)
            cv2.moveWindow(WIN_NAME, -10000, -10000)  # 校准时把悬浮窗挪开，免得挡住
            calibrate(capturer, cfg)
            cv2.moveWindow(WIN_NAME, *pos)
            tracker = Tracker(cfg)
        elif key in (ord("a"), ord("A")):
            if mini_abs is None:
                print("请先校准小地图位置（按 C）")
            else:
                try:
                    pos = cv2.getWindowImageRect(WIN_NAME)[:2]
                except Exception:
                    pos = (30, 200)
                cv2.moveWindow(WIN_NAME, -10000, -10000)  # 先挪开悬浮窗，免得挡住小地图
                cv2.waitKey(1)
                time.sleep(0.2)
                mini = capturer.grab(*mini_abs)
                if mini is None:
                    print("截图失败，请确认游戏窗口完整显示在屏幕上")
                else:
                    print("\n【校准朝向箭头】在弹出的放大图里，点一下小地图中间的玩家箭头")
                    calibrate_arrow(mini, cfg)
                cv2.moveWindow(WIN_NAME, *pos)
                tracker.arrow, tracker.arrow_fails = None, 0
        try:
            if cv2.getWindowProperty(WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            break

    try:
        x, y, _, _ = cv2.getWindowImageRect(WIN_NAME)
        if x > -5000:
            cfg["window_pos"] = [x, y]
    except Exception:
        pass
    cv2.destroyAllWindows()
    save_config(cfg)
    print("已退出，设置已保存。")


def main():
    ap = argparse.ArgumentParser(description="洛克王国世界 · 小地图实时定位（Windows 版）")
    ap.add_argument("--setup", action="store_true", help="重新运行设置向导")
    ap.add_argument("--calibrate", action="store_true", help="重新校准小地图位置")
    args = ap.parse_args()

    try:
        import mss  # noqa: F401
    except ImportError:
        print("缺少 mss 库，请先运行：pip install opencv-python numpy mss pillow")
        input("按回车退出")
        return
    if not IS_WIN:
        print("⚠ 当前不是 Windows，只能使用“截取整个显示器”模式。")
    elif "Win32 UI" not in cv2.getBuildInformation():
        print("✗ 当前 OpenCV 没有窗口功能（装成了 opencv-python-headless），悬浮窗无法显示。")
        print("  请在当前环境运行：")
        print("    pip uninstall -y opencv-python-headless opencv-contrib-python-headless")
        print("    pip install --force-reinstall opencv-python")
        input("按回车退出")
        return

    set_dpi_aware()
    cfg = load_config()
    if args.setup or not cfg.get("map_path") or not Path(cfg["map_path"]).exists():
        setup(cfg)
    run(cfg, force_calibrate=args.calibrate)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        input("\n程序出错，按回车退出（可把上面的报错发给我）")
