#!/usr/bin/env python3
"""
Guide-Aware Detailed Router — Python Implementation
"""

import argparse
import collections
import heapq
import time
import os

import LEFDEFParser
from LEFDEFParser import Rect


# ──────────────────────────────────────────────────────────────────────
# Technology tables — filled by extract_tech_from_lef_def()
# ──────────────────────────────────────────────────────────────────────
LAYERS   = []   # routing layers bottom→top
ADJ      = {}   # {layer: [adjacent_layers]}
DIR      = {}   # {layer: 'H' | 'V'}
WIDTH    = {}   # {layer: int}  minimum width in DEF units
SPACING  = {}   # {layer: int}  minimum spacing in DEF units

CHECKER_SPACING = {
    'li1':  170,
    'met1': 140,
    'met2': 140,
    'met3': 300,
    'met4': 300,
    'met5': 1600,
}

SKIP_NETS = {"clk", "VPWR", "VGND"}

SKIP_CELLS = {
    "sky130_fd_sc_hd__decap_3",  "sky130_fd_sc_hd__decap_4",
    "sky130_fd_sc_hd__decap_6",  "sky130_fd_sc_hd__decap_8",
    "sky130_fd_sc_hd__decap_12",
    "sky130_fd_sc_hd__fill_1",   "sky130_fd_sc_hd__fill_2",
    "sky130_fd_sc_hd__fill_4",   "sky130_fd_sc_hd__fill_8",
    "sky130_fd_sc_hd__lpflow_decapkapwr_3",
    "sky130_fd_sc_hd__lpflow_decapkapwr_4",
    "sky130_fd_sc_hd__lpflow_decapkapwr_6",
    "sky130_fd_sc_hd__lpflow_decapkapwr_8",
    "sky130_fd_sc_hd__lpflow_decapkapwr_12",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_1",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_2",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_hl_isowell_tap_4",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_1",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_2",
    "sky130_fd_sc_hd__lpflow_lsbuf_lh_isowell_tap_4",
    "sky130_fd_sc_hd__tap_1",    "sky130_fd_sc_hd__tap_2",
    "sky130_fd_sc_hd__tapvgnd2_1", "sky130_fd_sc_hd__tapvgnd_1",
    "sky130_fd_sc_hd__tapvpwrvgnd_1",
    "sky130_ef_sc_hd__decap_12",
}

BUCKET_SIZE       = 2000
LARGE_DESIGN_NETS = 300

# Cost constants — tuned so obstacle avoidance >> spacing avoidance >> guide preference
COST_OBSTACLE  = 10_000_000   # hard block: never route through obstacle
COST_SPACING   = 1_000_000    # strong discourage: spacing violation
COST_VIA       = 1_200        # layer-change penalty
COST_NONPREF   = 3            # multiplier for non-preferred direction
COST_OFFGUIDE  = 2_000        # penalty for being outside guide region


# ──────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────

def to_tuple(r):
    return (int(r.ll.x), int(r.ll.y), int(r.ur.x), int(r.ur.y))

def norm(r):
    x1, y1, x2, y2 = r
    return (min(x1,x2), min(y1,y2), max(x1,x2), max(y1,y2))

def overlap(a, b):
    return a[2] > b[0] and a[0] < b[2] and a[3] > b[1] and a[1] < b[3]

def bloat(r, d):
    return (r[0]-d, r[1]-d, r[2]+d, r[3]+d)

def ctr(r):
    return ((r[0]+r[2])//2, (r[1]+r[3])//2)

def area(r):
    return max(0, r[2]-r[0]) * max(0, r[3]-r[1])

def manhattan_xy(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def legal(layer, r):
    """Expand rect so both dimensions >= layer minimum width (checker minWidth test)."""
    x1, y1, x2, y2 = norm(r)
    w = WIDTH.get(layer, 140)
    if x2-x1 < w:
        c = (x1+x2)//2; x1 = c - w//2; x2 = x1 + w
    if y2-y1 < w:
        c = (y1+y2)//2; y1 = c - w//2; y2 = y1 + w
    return (int(x1), int(y1), int(x2), int(y2))

def box(layer, x, y):
    h = WIDTH.get(layer, 140)//2
    return legal(layer, (x-h, y-h, x+h, y+h))

def hwire(layer, x1, x2, y):
    h = WIDTH.get(layer, 140)//2
    return legal(layer, (min(x1,x2), y-h, max(x1,x2), y+h))

def vwire(layer, x, y1, y2):
    h = WIDTH.get(layer, 140)//2
    return legal(layer, (x-h, min(y1,y2), x+h, max(y1,y2)))


# ──────────────────────────────────────────────────────────────────────
# RMST — Rectilinear Minimum Spanning Tree (Prim's on Manhattan metric)
# ──────────────────────────────────────────────────────────────────────

def rmst_edges(points):
    """Decompose multi-pin net into 2-pin edges via greedy Prim's RMST."""
    pts = list(dict.fromkeys(points))
    if len(pts) <= 1:
        return []
    if len(pts) == 2:
        return [(pts[0], pts[1])]

    remaining = pts[:]
    root = min(remaining, key=lambda p: (p[0], p[1]))
    remaining.remove(root)
    tree  = [root]
    edges = []

    while remaining:
        best_dist, best_pair = 10**30, None
        for p in remaining:
            for t in tree:
                d = manhattan_xy(p, t)
                if d < best_dist:
                    best_dist = d; best_pair = (t, p)
        a, b = best_pair
        edges.append((a, b))
        tree.append(b); remaining.remove(b)

    return edges


# ──────────────────────────────────────────────────────────────────────
# Technology extraction from LEF text + DEFReader
# ──────────────────────────────────────────────────────────────────────

def _safe_call(obj, name, default=None):
    if not hasattr(obj, name):
        return default
    try:
        v = getattr(obj, name)
        return v() if callable(v) else v
    except Exception:
        return default


def _parse_lef_text(input_lef):
    """Parse routing layer properties and VIA adjacency from LEF text."""
    layer_order, props, via_edges = [], {}, set()
    try:
        lines = open(input_lef, errors="ignore").read().splitlines()
    except Exception:
        return layer_order, props, via_edges

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        toks = line.replace(";", " ").split()

        if len(toks) >= 2 and toks[0].upper() == "LAYER":
            lname = toks[1]; block = []; i += 1
            while i < len(lines):
                l2 = lines[i].strip()
                if l2.upper().startswith("END"): break
                block.append(l2); i += 1
            text = "\n".join(block)
            if "TYPE ROUTING" in text.upper():
                if lname not in layer_order:
                    layer_order.append(lname)
                direction = width = spacing = None
                in_st = False
                for b in block:
                    parts = b.replace(";", " ").split()
                    if not parts: continue
                    key = parts[0].upper()
                    if key == "DIRECTION" and len(parts) >= 2:
                        d = parts[1].upper()
                        direction = "H" if d.startswith("HOR") else ("V" if d.startswith("VER") else None)
                    elif key == "WIDTH" and len(parts) >= 2 and width is None and not in_st:
                        try: width = int(round(float(parts[1]) * 1000))
                        except: pass
                    elif key == "SPACING" and len(parts) >= 2 and spacing is None and not in_st:
                        try: spacing = int(round(float(parts[1]) * 1000))
                        except: pass
                    elif key == "SPACINGTABLE":
                        in_st = True
                    elif in_st and key == "WIDTH" and len(parts) >= 3 and spacing is None:
                        try: spacing = int(round(float(parts[2]) * 1000))
                        except: pass
                    elif in_st and key not in ("WIDTH", "PARALLELRUNLENGTH", ""):
                        in_st = False
                props[lname] = {"direction": direction, "width": width, "spacing": spacing}

        elif len(toks) >= 2 and toks[0].upper() == "VIA":
            via_layers = []; i += 1
            while i < len(lines):
                l2 = lines[i].strip(); parts = l2.replace(";", " ").split()
                if len(parts) >= 2 and parts[0].upper() == "LAYER":
                    via_layers.append(parts[1])
                if l2.upper().startswith("END"): break
                i += 1
            routing = [l for l in via_layers if l in props or l in layer_order]
            for a, b in zip(routing, routing[1:]):
                if a != b: via_edges.add(tuple(sorted((a, b))))
        i += 1

    return layer_order, props, via_edges


def extract_tech_from_lef_def(lef_reader, def_reader, input_lef):
    """Populate global LAYERS/ADJ/DIR/WIDTH/SPACING from LEF/DEF.
    SPACING values are overridden by CHECKER_SPACING to exactly match checker.py."""
    global LAYERS, ADJ, DIR, WIDTH, SPACING

    lef_order, lef_props, via_edges = _parse_lef_text(input_lef)

    try:
        track_layers = set(def_reader.tracks().keys())
    except Exception:
        track_layers = set(lef_order)

    ordered = [l for l in lef_order if not track_layers or l in track_layers]
    if not ordered:
        ordered = list(track_layers)
    LAYERS = ordered

    parser_width = {}
    try:
        for layer in lef_reader.layers():
            lname = layer.name()
            w = _safe_call(layer, "width", None)
            if w is not None and int(w) > 0:
                parser_width[lname] = int(w)
    except Exception:
        pass

    WIDTH = {}; SPACING = {}; DIR = {}

    for lname in LAYERS:
        p = lef_props.get(lname, {})
        w = parser_width.get(lname) or p.get("width") or 140
        # Use EXACT checker spacing — this is the single most important calibration
        sp = CHECKER_SPACING.get(lname) or p.get("spacing") or w
        d = p.get("direction")
        if d is None:
            xcnt = ycnt = 0
            for t in def_reader.tracks().get(lname, []):
                orient = _safe_call(t, "orient", None)
                if orient == "X": xcnt += 1
                elif orient == "Y": ycnt += 1
            d = "V" if xcnt >= ycnt else "H"
        WIDTH[lname]   = int(w)
        SPACING[lname] = int(sp)
        DIR[lname]     = d

    ADJ = {l: [] for l in LAYERS}
    for a, b in via_edges:
        if a in ADJ and b in ADJ:
            if b not in ADJ[a]: ADJ[a].append(b)
            if a not in ADJ[b]: ADJ[b].append(a)
    for i, l in enumerate(LAYERS):
        if i > 0 and LAYERS[i-1] not in ADJ[l]:
            ADJ[l].append(LAYERS[i-1])
        if i+1 < len(LAYERS) and LAYERS[i+1] not in ADJ[l]:
            ADJ[l].append(LAYERS[i+1])

    print("[TECH] layers =", LAYERS)
    print("[TECH] dir    =", DIR)
    print("[TECH] width  =", WIDTH)
    print("[TECH] spacing=", SPACING)


def layer_path(src, dst):
    """Shortest via-adjacency path between two routing layers (BFS)."""
    if src == dst: return [src]
    if src not in ADJ or dst not in ADJ: return [src]
    q = collections.deque([src]); parent = {src: None}
    while q:
        cur = q.popleft()
        for nb in ADJ.get(cur, []):
            if nb in parent: continue
            parent[nb] = cur
            if nb == dst:
                path = [dst]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                return path[::-1]
            q.append(nb)
    return [src]


# ──────────────────────────────────────────────────────────────────────
# GUIDE file parser
# ──────────────────────────────────────────────────────────────────────

def parse_guide(path):
    """Parse global routing GUIDE file → {net_name: [(x1,y1,x2,y2,layer), ...]}"""
    guide = collections.defaultdict(list)
    if not path or not os.path.exists(path):
        return guide
    lines = [ln.strip() for ln in open(path, errors="ignore") if ln.strip()]
    i = 0
    while i < len(lines):
        name = lines[i]; i += 1
        if i >= len(lines) or lines[i] != "(": continue
        i += 1
        while i < len(lines) and lines[i] != ")":
            p = lines[i].split()
            if len(p) == 5 and p[4] in LAYERS:
                x1, y1, x2, y2 = map(int, p[:4])
                guide[name].append((min(x1,x2), min(y1,y2), max(x1,x2), max(y1,y2), p[4]))
            i += 1
        i += 1
    return guide


# ──────────────────────────────────────────────────────────────────────
# Instance geometry (transformed pin/obstruction rects)
# ──────────────────────────────────────────────────────────────────────

class Inst:
    def __init__(self, comp, macro):
        origin = comp.location()
        self.pins  = {}
        self.obsts = collections.defaultdict(list)
        for p in macro.pins():
            pname = p.name()
            self.pins[pname] = collections.defaultdict(list)
            for port in p.ports():
                for layer, rects in port.items():
                    if layer not in LAYERS: continue
                    for v in rects:
                        r = Rect(v.ll.x, v.ll.y, v.ur.x, v.ur.y)
                        r.transform(comp.orient(), origin, macro.xdim(), macro.ydim())
                        self.pins[pname][layer].append(to_tuple(r))
        for layer, rects in macro.obstructions().items():
            if layer not in LAYERS: continue
            for v in rects:
                r = Rect(v.ll.x, v.ll.y, v.ur.x, v.ur.y)
                r.transform(comp.orient(), origin, macro.xdim(), macro.ydim())
                self.obsts[layer].append(to_tuple(r))


# ──────────────────────────────────────────────────────────────────────
# Design extraction
# ──────────────────────────────────────────────────────────────────────

def extract_design(input_def, input_lef, input_guide):
    lef_reader = LEFDEFParser.LEFReader()
    def_reader = LEFDEFParser.DEFReader()
    lef_reader.readLEF(input_lef)
    def_reader.readDEF(input_def)
    extract_tech_from_lef_def(lef_reader, def_reader, input_lef)

    macros = {m.name(): m for m in lef_reader.macros()}
    bbox   = def_reader.bbox()
    die    = (bbox.ll.x, bbox.ll.y, bbox.ur.x, bbox.ur.y)

    # Build track coordinate sets per layer
    tracks = {layer: {"x": set(), "y": set()} for layer in LAYERS}
    for layer, grid_list in def_reader.tracks().items():
        if layer not in tracks: continue
        for t in grid_list:
            if t.orient == "X":
                for i in range(t.num): tracks[layer]["x"].add(t.x + i*t.step)
            elif t.orient == "Y":
                for i in range(t.num): tracks[layer]["y"].add(t.x + i*t.step)
    for layer in LAYERS:
        tracks[layer]["x"] = sorted(tracks[layer]["x"])
        tracks[layer]["y"] = sorted(tracks[layer]["y"])

    insts = {}
    for comp in def_reader.components():
        if comp.macro() in SKIP_CELLS or comp.macro() not in macros: continue
        insts[comp.name()] = Inst(comp, macros[comp.macro()])

    boundary_pins = {}
    for p in def_reader.pins():
        pname = p.name()
        boundary_pins[pname] = collections.defaultdict(list)
        for port in p.ports():
            for layer, rects in port.items():
                if layer not in LAYERS: continue
                for r in rects:
                    boundary_pins[pname][layer].append(to_tuple(r))

    nets = {}
    used = set()
    for net in def_reader.nets():
        nname = net.name()
        if nname in SKIP_NETS: continue
        groups = []
        for cell, pin in net.pins():
            used.add((cell, pin))
            if cell == "PIN" and pin in boundary_pins:
                groups.append((cell, pin, boundary_pins[pin]))
            elif cell in insts and pin in insts[cell].pins:
                groups.append((cell, pin, insts[cell].pins[pin]))
        if len(groups) >= 2:
            nets[nname] = groups

    # Obstacles = macro OBS + unused pins (mirrors checker.py markUnusedPins)
    obstacles = collections.defaultdict(list)
    for inst in insts.values():
        for layer, rects in inst.obsts.items():
            obstacles[layer].extend(rects)

    used_boundary = {pin for cell, pin in used if cell == "PIN"}
    for pname, lr in boundary_pins.items():
        if pname not in used_boundary:
            for layer, rects in lr.items():
                obstacles[layer].extend(rects)

    for inst_name, inst in insts.items():
        for pin_name, lr in inst.pins.items():
            if (inst_name, pin_name) not in used:
                for layer, rects in lr.items():
                    obstacles[layer].extend(rects)

    guides = parse_guide(input_guide)
    return die, tracks, nets, obstacles, guides


# ──────────────────────────────────────────────────────────────────────
# Detailed Router
# ──────────────────────────────────────────────────────────────────────

class DetailedRouter:

    def __init__(self, die, tracks, nets, obstacles, guides):
        self.die    = die
        self.tracks = tracks
        self.nets   = nets
        self.guides = guides

        self.routes    = collections.defaultdict(list)
        self.occ       = collections.defaultdict(list)
        self.occ_index = collections.defaultdict(list)

        # Separate obstacle index for fast hard-block query
        self.obs_index = collections.defaultdict(list)

        for layer, rects in obstacles.items():
            for r in rects:
                self._add_occ(layer, "obst", r)
                for key in self._bucket_keys(layer, norm(r)):
                    self.obs_index[key].append(norm(r))

        for net, groups in nets.items():
            for _, _, lr in groups:
                for layer, rects in lr.items():
                    for r in rects:
                        self._add_occ(layer, net, r)

    # ── Spatial bucket index ──────────────────────────────────────────

    def _bucket_keys(self, layer, r):
        x1, y1, x2, y2 = r
        for bx in range(x1//BUCKET_SIZE, x2//BUCKET_SIZE+1):
            for by in range(y1//BUCKET_SIZE, y2//BUCKET_SIZE+1):
                yield (layer, bx, by)

    def _add_occ(self, layer, owner, r):
        r = norm(r)
        self.occ[layer].append((owner, r))
        for key in self._bucket_keys(layer, r):
            self.occ_index[key].append((owner, r))

    def _nearby_occ(self, layer, r):
        sp = SPACING.get(layer, 140)
        search = bloat(r, sp)          # ← exactly 1*spacing (checker exact match)
        seen, out = set(), []
        for key in self._bucket_keys(layer, search):
            for owner, o in self.occ_index.get(key, []):
                oid = (owner, o)
                if oid not in seen:
                    seen.add(oid); out.append((owner, o))
        return out

    def _hits_obstacle(self, layer, r):
        """Fast hard-block check: does shape r overlap any obstacle on layer?"""
        rn = norm(r)
        for key in self._bucket_keys(layer, rn):
            for o in self.obs_index.get(key, []):
                if overlap(rn, o):
                    return True
        return False

    # ── DRC conflict counter ─────────────────────────────────────────

    def count_bad(self, net, shapes):
        """
        Count DRC violations:
         - MinWidth: shape dimension < layerWidth  (→ 1 violation)
         - Spacing: bloated shape overlaps different-net shape on same layer
         - Obstacle hit: counted as high-cost but separate from spacing
         - Boundary: shape outside die area
        """
        cnt = 0
        dx1, dy1, dx2, dy2 = self.die
        for layer, r in shapes:
            r  = legal(layer, r)
            sp = SPACING.get(layer, 140)
            rb = bloat(r, sp)
            # Boundary
            if r[0] < dx1 or r[1] < dy1 or r[2] > dx2 or r[3] > dy2:
                cnt += 1000
            for owner, other in self._nearby_occ(layer, r):
                if owner == net: continue
                if overlap(rb, other):
                    if owner == "obst":
                        cnt += COST_OBSTACLE   # hard block — never acceptable
                    else:
                        cnt += 1               # spacing violation against another net
        return cnt

    # ── Layer helpers ─────────────────────────────────────────────────

    def _usable_layers(self):
        """Exclude very-wide strap layers (e.g. met5) without hardcoding names."""
        if not LAYERS: return []
        widths    = sorted(WIDTH.get(l, 140) for l in LAYERS)
        threshold = max(3*widths[0], widths[min(len(widths)-1, 3)])
        out = [l for l in LAYERS if WIDTH.get(l, 140) <= threshold]
        return out or LAYERS[:]

    def _preferred_layers(self, avoid_layers=None):
        """
        Return routing layers in preference order.
        FIX: prefer met1/met2 over li1 because li1 is heavily congested
        with cell metal. Routing on li1 always risks spacing hits with OBS.
        """
        avoid_layers = avoid_layers or set()
        usable = self._usable_layers()
        out = [l for l in usable if l not in avoid_layers]
        if not out:
            out = usable[:]
        # Move li1 (index 0) to end — prefer higher metal layers first
        if len(out) > 1 and out[0] == LAYERS[0]:
            out = out[1:] + [out[0]]
        return out

    def _preferred_trunk(self, avoid_layers=None):
        avoid_layers = avoid_layers or set()
        usable = self._usable_layers()
        h_layers = [l for l in usable if DIR.get(l) == "H" and l not in avoid_layers]
        if h_layers:
            return h_layers[len(h_layers)//2]
        for l in usable:
            if l not in avoid_layers: return l
        return usable[0] if usable else None

    # ── Track snapping ────────────────────────────────────────────────

    def nearest_track(self, layer, x, y):
        xs = self.tracks.get(layer, {}).get("x", [])
        ys = self.tracks.get(layer, {}).get("y", [])
        sx = min(xs, key=lambda v: abs(v-x)) if xs else x
        sy = min(ys, key=lambda v: abs(v-y)) if ys else y
        return sx, sy

    def nearest_track_in_rect(self, layer, r):
        """Find nearest track intersection that lies INSIDE rect r.
        Returns (x, y) or None if no track lies inside."""
        x1, y1, x2, y2 = r
        xs = self.tracks.get(layer, {}).get("x", [])
        ys = self.tracks.get(layer, {}).get("y", [])
        cx, cy = (x1+x2)//2, (y1+y2)//2

        # Filter to tracks inside the rect
        xs_in = [v for v in xs if x1 <= v <= x2] or \
                sorted(xs, key=lambda v: min(abs(v-x1), abs(v-x2)))[:1]
        ys_in = [v for v in ys if y1 <= v <= y2] or \
                sorted(ys, key=lambda v: min(abs(v-y1), abs(v-y2)))[:1]

        if not xs_in or not ys_in:
            return None
        bx = min(xs_in, key=lambda v: abs(v-cx))
        by = min(ys_in, key=lambda v: abs(v-cy))
        return bx, by

    # ── Guide-region query ────────────────────────────────────────────

    def _guide_regions(self, net, pad=0):
        by_layer = collections.defaultdict(list)
        for x1, y1, x2, y2, layer in self.guides.get(net, []):
            if layer in LAYERS:
                by_layer[layer].append((x1-pad, y1-pad, x2+pad, y2+pad))
        return by_layer

    def _guide_cost(self, net, layer, r):
        if not self.guides.get(net): return 0
        rr = norm(r)
        for g in self._guide_regions(net).get(layer, []):
            if overlap(rr, g): return 0
        return COST_OFFGUIDE

    # ── Shape construction ────────────────────────────────────────────

    def _track_connector(self, layer, px, py):
        """
        Connect exact pin point (px,py) to nearest track on layer.
        Strategy:
          - Snap (px,py) to (sx,sy) on nearest track.
          - Place box at (px,py) [covers pin rect for connectivity].
          - Place box at (sx,sy) [A* start point].
          - If different: emit the shortest on-track wire between them.
        """
        sx, sy = self.nearest_track(layer, px, py)
        shapes = [(layer, box(layer, px, py)),
                  (layer, box(layer, sx, sy))]
        if sx == px and sy == py:
            return shapes
        # Only emit connecting wire if BOTH endpoints are on track axes
        tx_set = set(self.tracks.get(layer, {}).get("x", []))
        ty_set = set(self.tracks.get(layer, {}).get("y", []))
        if px == sx:   # purely vertical move — both X on same track → ok
            shapes.append((layer, vwire(layer, px, py, sy)))
        elif py == sy: # purely horizontal move — both Y on same track → ok
            shapes.append((layer, hwire(layer, px, sx, py)))
        else:
            # Off-grid pin: emit L-shape only if corner is also on-track
            # Prefer horizontal first on H layers, vertical first on V layers
            if DIR.get(layer) == "H":
                # corner at (sx, py): sx is on-track X, py is pin Y (may be off)
                # Only add if py is a track Y (or very close)
                closest_y = min(ty_set, key=lambda v: abs(v-py)) if ty_set else py
                if abs(closest_y - py) <= WIDTH.get(layer, 140):
                    shapes.append((layer, hwire(layer, px, sx, closest_y)))
                    shapes.append((layer, box(layer, sx, closest_y)))
                    shapes.append((layer, vwire(layer, sx, closest_y, sy)))
                else:
                    # Fall back: vertical first to a track Y near py, then horizontal
                    shapes.append((layer, vwire(layer, px, py, sy)))
                    shapes.append((layer, box(layer, px, sy)))
                    shapes.append((layer, hwire(layer, px, sx, sy)))
            else:
                closest_x = min(tx_set, key=lambda v: abs(v-px)) if tx_set else px
                if abs(closest_x - px) <= WIDTH.get(layer, 140):
                    shapes.append((layer, vwire(layer, closest_x, py, sy)))
                    shapes.append((layer, box(layer, closest_x, sy)))
                    shapes.append((layer, hwire(layer, closest_x, sx, sy)))
                else:
                    shapes.append((layer, hwire(layer, px, sx, py)))
                    shapes.append((layer, box(layer, sx, py)))
                    shapes.append((layer, vwire(layer, sx, py, sy)))
        return shapes

    def _path_to_shapes(self, path):
        """
        Convert A* node path → DEF rectangles.
        """
        shapes = []
        if not path: return shapes

        for layer, x, y in path:
            shapes.append((layer, box(layer, x, y)))

        for u, v in zip(path, path[1:]):
            if u[0] == v[0]:
                layer = u[0]
                if u[1] == v[1]:
                    shapes.append((layer, vwire(layer, u[1], u[2], v[2])))
                elif u[2] == v[2]:
                    shapes.append((layer, hwire(layer, u[1], v[1], u[2])))
                else:
                    # Off-grid diagonal fallback (should be rare with on-track A*)
                    shapes.append((layer, hwire(layer, u[1], v[1], u[2])))
                    shapes.append((layer, box(layer, v[1], u[2])))
                    shapes.append((layer, vwire(layer, v[1], u[2], v[2])))
            else:
               
                x, y = u[1], u[2]  # both nodes snapped to same (x,y) at via point
                shapes.append((u[0], box(u[0], x, y)))
                shapes.append((v[0], box(v[0], x, y)))

        return shapes

    def _via_stack(self, x, y, layer_a, layer_b):
        """Boxes on all routing layers between layer_a and layer_b at (x,y)."""
        if layer_a not in LAYERS or layer_b not in LAYERS: return []
        return [(l, box(l, x, y)) for l in layer_path(layer_a, layer_b) if l in LAYERS]

    # ── Last-resort L-wire fallback ───────────────────────────────────

    def _direct_lwire(self, net, x1, y1, x2, y2):
        """Guaranteed L-wire fallback — tries all usable layers."""
        usable = self._usable_layers() or LAYERS[:]
        best = None
        for layer in usable:
            sx1, sy1 = self.nearest_track(layer, x1, y1)
            sx2, sy2 = self.nearest_track(layer, x2, y2)
            sh = [(layer, box(layer, sx1, sy1)),
                  (layer, hwire(layer, sx1, sx2, sy1)),
                  (layer, box(layer, sx2, sy1)),
                  (layer, vwire(layer, sx2, sy1, sy2)),
                  (layer, box(layer, sx2, sy2))]
            bad = self.count_bad(net, sh)
            if best is None or bad < best[0]:
                best = (bad, sh)
                if bad == 0: break
        return best[1] if best else []

    # ── Pin access ───────────────────────────────────────────────────

    def _pin_candidates(self, lr):
        """
        Enumerate candidate access points inside pin rectangles.
        Prioritise track-intersection points INSIDE the pin rect.
        A track point inside the pin means zero-length connector → no off-track shapes.
        """
        out, seen = [], set()
        for layer, rects in lr.items():
            for r in sorted(rects, key=area, reverse=True):
                x1, y1, x2, y2 = r
                w = WIDTH.get(layer, 140)

                # Priority 1: track intersection inside pin rect
                tp = self.nearest_track_in_rect(layer, r)
                if tp:
                    pt = (layer, tp[0], tp[1])
                    if pt not in seen:
                        seen.add(pt); out.append(pt)

                # Priority 2: track-snapped center (may be outside rect)
                cx, cy = (x1+x2)//2, (y1+y2)//2
                sx, sy = self.nearest_track(layer, cx, cy)
                sx = max(x1, min(x2, sx)); sy = max(y1, min(y2, sy))
                for px, py in [(sx, sy), (cx, cy),
                               (x1+w//2, cy), (x2-w//2, cy),
                               (cx, y1+w//2), (cx, y2-w//2)]:
                    if x1 <= px <= x2 and y1 <= py <= y2:
                        pt = (layer, px, py)
                        if pt not in seen:
                            seen.add(pt); out.append(pt)
        return out

    def _access_shapes(self, pin_layer, x, y, avoid_layers=None):
        """
        Create pin-access shapes.
        Only place box on pin_layer itself (not a full via stack to trunk).
        Multi-layer access stacks caused spacing violations on intermediate layers.
        The A* router will add the actual via as needed during routing.
        """
        avoid_layers = avoid_layers or set()
        if pin_layer in avoid_layers:
            # If pin layer is avoided, try an adjacent non-avoided layer
            cands = [l for l in ADJ.get(pin_layer, []) if l in LAYERS and l not in avoid_layers]
            use_layer = cands[0] if cands else pin_layer
        else:
            use_layer = pin_layer
        return [(use_layer, box(use_layer, x, y))]

    def _choose_accesses(self, net, groups, avoid_layers=None):
        """Pick best access point per pin, ordered by RMST."""
        avoid_layers = avoid_layers or set()
        result, temp = [], []

        for _, _, lr in groups:
            best = None
            for pin_layer, x, y in self._pin_candidates(lr):
                stack  = self._access_shapes(pin_layer, x, y, avoid_layers)
                sc     = self.count_bad(net, stack + temp)
                n_avoid = sum(1 for l, _ in stack if l in avoid_layers)
                metric = (sc, n_avoid, len(stack))
                if best is None or metric < best[0]:
                    best = (metric, (x, y, stack))
                    if sc == 0 and n_avoid == 0: break
            if best:
                x, y, stack = best[1]
                temp.extend(stack)
                result.append((x, y, stack))

        # Re-order by RMST
        pts = [(x, y) for x, y, _ in result]
        pos_map = {(x, y): (x, y, st) for x, y, st in result}
        ordered = []
        seen = set()
        if pts:
            first = min(pts, key=lambda p: (p[0], p[1]))
            ordered.append(first); seen.add(first)
        for a, b in rmst_edges(pts):
            for p in (a, b):
                if p not in seen:
                    ordered.append(p); seen.add(p)
        return [pos_map[p] for p in ordered if p in pos_map] or result

    # ── Core A* router (one 2-pin edge) ──────────────────────────────

    def _astar_edge(self, net, a, b, avoid_layers=None):
        """
        A* route one 2-pin edge (ax,ay)→(bx,by).

        Cost function:
          - Step cost: Manhattan distance, ×COST_NONPREF for non-preferred direction
          - Obstacle hit: COST_OBSTACLE (hard block — effectively infinite)
          - Spacing violation: COST_SPACING per conflict
          - Off-guide: COST_OFFGUIDE
          - Via (layer change): COST_VIA

        Obstacle cost is now COST_OBSTACLE (10M) not 1000.
        This ensures A* never routes through cell obstructions.
        """
        avoid_layers = avoid_layers or set()
        ax, ay = a;  bx, by = b

        guide_layers = [g[4] for g in self.guides.get(net, []) if g[4] in LAYERS]
        normal_layers = self._preferred_layers(avoid_layers)
        allowed = list(dict.fromkeys(
            [l for l in guide_layers if l not in avoid_layers and
             WIDTH.get(l,140) <= max((WIDTH.get(x,140) for x in normal_layers), default=140)]
            + normal_layers
        )) or LAYERS[:]

        # Routing window
        gx_all, gy_all = [ax, bx], [ay, by]
        for x1, y1, x2, y2, _ in self.guides.get(net, []):
            gx_all += [x1, x2]; gy_all += [y1, y2]
        margin = max(3000, 6*max(WIDTH.values() or [140]))
        wx1 = max(self.die[0], min(gx_all)-margin)
        wx2 = min(self.die[2], max(gx_all)+margin)
        wy1 = max(self.die[1], min(gy_all)-margin)
        wy2 = min(self.die[3], max(gy_all)+margin)

        track_limit = 96 if len(self.nets) < LARGE_DESIGN_NETS else 48

        candidates = {}; node_set = set()
        for layer in allowed:
            xs = [v for v in self.tracks.get(layer,{}).get("x",[]) if wx1<=v<=wx2]
            ys = [v for v in self.tracks.get(layer,{}).get("y",[]) if wy1<=v<=wy2]
            sax, say = self.nearest_track(layer, ax, ay)
            sbx, sby = self.nearest_track(layer, bx, by)
            xs = sorted(set(xs+[sax,sbx])); ys = sorted(set(ys+[say,sby]))
            px, py = (ax+bx)//2, (ay+by)//2
            xs = sorted(xs, key=lambda v: min(abs(v-ax),abs(v-bx),abs(v-px)))[:track_limit]
            ys = sorted(ys, key=lambda v: min(abs(v-ay),abs(v-by),abs(v-py)))[:track_limit]
            candidates[layer] = (sorted(xs), sorted(ys))
            for x in candidates[layer][0]:
                for y in candidates[layer][1]:
                    node_set.add((layer, x, y))

        goals = set(); starts = []
        for layer in allowed:
            sax, say = self.nearest_track(layer, ax, ay)
            sbx, sby = self.nearest_track(layer, bx, by)
            if (layer, sax, say) in node_set: starts.append((layer, sax, say))
            if (layer, sbx, sby) in node_set: goals.add((layer, sbx, sby))

        if not starts or not goals: return []

        def heuristic(n):
            _, x, y = n
            return min(abs(x-gx)+abs(y-gy) for _, gx, gy in goals)

        def neighbors(n):
            layer, x, y = n
            xs, ys = candidates.get(layer, ([], []))
            out = []
            xi = xs.index(x) if x in xs else -1
            yi = ys.index(y) if y in ys else -1
            steps = (1, 2, 3) if len(self.nets) < LARGE_DESIGN_NETS else (1, 2)
            for k in steps:
                if xi-k >= 0:      out.append((layer, xs[xi-k], y))
                if xi+k < len(xs): out.append((layer, xs[xi+k], y))
                if yi-k >= 0:      out.append((layer, x, ys[yi-k]))
                if yi+k < len(ys): out.append((layer, x, ys[yi+k]))
            for nl in ADJ.get(layer, []):
                if nl in candidates:
                    sx, sy = self.nearest_track(nl, x, y)
                    nb = (nl, sx, sy)
                    if nb in node_set: out.append(nb)
            return out

        def seg_shape(layer, n1, n2):
            _, x1, y1 = n1; _, x2, y2 = n2
            if x1==x2 and y1==y2: return (layer, box(layer, x1, y1))
            if abs(x1-x2) >= abs(y1-y2): return (layer, hwire(layer, x1, x2, y1))
            return (layer, vwire(layer, x1, y1, y2))

        pq = []; dist = {}; parent = {}; bad_cache = {}

        def cached_bad(sh_list):
            key = tuple((l, r) for l, r in sh_list)
            v = bad_cache.get(key)
            if v is None:
                v = self.count_bad(net, sh_list); bad_cache[key] = v
            return v

        for st in starts[:min(4, len(starts))]:
            dist[st] = 0
            heapq.heappush(pq, (heuristic(st), 0, st))

        found = None; expanded = 0
        limit = 80000 if len(self.nets) < LARGE_DESIGN_NETS else 25000

        while pq and expanded < limit:
            _, g, cur = heapq.heappop(pq)
            if g != dist.get(cur): continue
            expanded += 1
            if cur in goals: found = cur; break

            for nb in neighbors(cur):
                layer = cur[0]; move_layer = nb[0]
                if layer == move_layer:
                    sl, sr = seg_shape(layer, cur, nb)
                    bad    = cached_bad([(sl, sr)])
                    step   = manhattan_xy((cur[1],cur[2]),(nb[1],nb[2]))
                    if cur[1]!=nb[1] and DIR.get(layer)=="V": step *= COST_NONPREF
                    if cur[2]!=nb[2] and DIR.get(layer)=="H": step *= COST_NONPREF
                    cost = step + COST_SPACING*bad + self._guide_cost(net, sl, sr)
                else:
                    via_sh = [(layer, box(layer, cur[1], cur[2])),
                              (move_layer, box(move_layer, nb[1], nb[2]))]
                    bad  = cached_bad(via_sh)
                    cost = COST_VIA + COST_SPACING*bad

                ng = g + cost
                if ng < dist.get(nb, 10**30):
                    dist[nb] = ng; parent[nb] = cur
                    heapq.heappush(pq, (ng+heuristic(nb), ng, nb))

        if found is None: return []

        path = [found]
        while path[-1] in parent: path.append(parent[path[-1]])
        path.reverse()

        shapes = []
        shapes.extend(self._track_connector(path[0][0], ax, ay))
        shapes.extend(self._track_connector(path[-1][0], bx, by))
        shapes.extend(self._path_to_shapes(path))
        return shapes

    # ── Primary A* net router ─────────────────────────────────────────

    def _route_net_astar(self, net, groups, avoid_layers=None):
        avoid_layers = avoid_layers or set()
        accesses = self._choose_accesses(net, groups, avoid_layers)
        if len(accesses) < 2: return []

        points = [(x, y) for x, y, _ in accesses]
        edges  = rmst_edges(points)
        if not edges: return []

        shapes = []
        for _, _, stack in accesses:
            shapes.extend(stack)

        pos_map = {(x, y): (x, y, st) for x, y, st in accesses}

        for pa, pb in edges:
            edge_shapes = self._astar_edge(net, pa, pb, avoid_layers)
            if not edge_shapes: return []
            shapes.extend(edge_shapes)

        return shapes

    # ── Connectivity check (mirrors checker.py) ───────────────────────

    def _net_components(self, net, shapes):
        items = []
        for layer, r in shapes:
            if layer in LAYERS:
                items.append((layer, legal(layer, r), 'route'))
        for _, _, lr in self.nets.get(net, []):
            for layer, rects in lr.items():
                if layer not in LAYERS: continue
                for r in rects:
                    items.append((layer, norm(r), 'pin'))

        n = len(items)
        if n <= 1: return [items] if items else []

        parent = list(range(n))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]; x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb: parent[rb] = ra

        for i in range(n):
            li, ri, _ = items[i]
            for j in range(i+1, n):
                lj, rj, _ = items[j]
                if li == lj:
                    # same layer: touch within 1 DBU (matches checker addNeighbours)
                    if (ri[2]+1>rj[0] and ri[0]-1<rj[2] and
                            ri[3]+1>rj[1] and ri[1]-1<rj[3]):
                        union(i, j)
                elif lj in ADJ.get(li, []) or li in ADJ.get(lj, []):
                    if overlap(ri, rj): union(i, j)

        comps = collections.defaultdict(list)
        for i, it in enumerate(items):
            comps[find(i)].append(it)
        return list(comps.values())

    # ── A*-based connectivity repair ──────────────────────────────────

    def _closest_pair(self, comp_a, comp_b):
        best = None
        def rank(a, b):
            return 0 if a[2]=='route' and b[2]=='route' else (1 if a[2]=='route' or b[2]=='route' else 2)
        for ia in comp_a:
            la, ra, _ = ia; ca = ctr(ra)
            for ib in comp_b:
                lb, rb, _ = ib; cb = ctr(rb)
                dx = max(0, max(ra[0],rb[0]) - min(ra[2],rb[2]))
                dy = max(0, max(ra[1],rb[1]) - min(ra[3],rb[3]))
                score = (rank(ia,ib), dx+dy, manhattan_xy(ca,cb))
                if best is None or score < best[0]:
                    best = (score, ia, ib)
        if best is None: return None, None
        _, ia, ib = best
        return ia, ib

    def _connect_components(self, comp_a, comp_b, net):
        """A*-based repair: connect two disconnected checker components."""
        ia, ib = self._closest_pair(comp_a, comp_b)
        if ia is None or ib is None: return []
        la, ra, _ = ia; lb, rb, _ = ib
        xa, ya = ctr(ra); xb, yb = ctr(rb)

        allowed = self._preferred_layers()
        low = LAYERS[0] if LAYERS else None
        if len(allowed) > 1 and low in allowed:
            allowed = [l for l in allowed if l != low] or allowed

        # FIX: doubled margin for repair so A* has more room to find clean detours
        margin = max(8000, 12*max(WIDTH.values() or [140]))
        wx1 = max(self.die[0], min(xa,xb)-margin)
        wx2 = min(self.die[2], max(xa,xb)+margin)
        wy1 = max(self.die[1], min(ya,yb)-margin)
        wy2 = min(self.die[3], max(ya,yb)+margin)

        repair_limit = 140 if len(self.nets) < LARGE_DESIGN_NETS else 80
        candidates = {}; node_set = set()
        for layer in allowed:
            xs = [v for v in self.tracks.get(layer,{}).get("x",[]) if wx1<=v<=wx2]
            ys = [v for v in self.tracks.get(layer,{}).get("y",[]) if wy1<=v<=wy2]
            for ex, ey in [(xa,ya),(xb,yb)]:
                sx, sy = self.nearest_track(layer, ex, ey)
                xs.append(sx); ys.append(sy)
            pm = (xa+xb)//2, (ya+yb)//2
            xs = sorted(set(xs), key=lambda v: min(abs(v-xa),abs(v-xb),abs(v-pm[0])))[:repair_limit]
            ys = sorted(set(ys), key=lambda v: min(abs(v-ya),abs(v-yb),abs(v-pm[1])))[:repair_limit]
            candidates[layer] = (sorted(xs), sorted(ys))
            for x in candidates[layer][0]:
                for y in candidates[layer][1]:
                    node_set.add((layer, x, y))

        starts = []; goals = set()
        for layer in allowed:
            sx, sy = self.nearest_track(layer, xa, ya)
            gx, gy = self.nearest_track(layer, xb, yb)
            if (layer, sx, sy) in node_set: starts.append((layer, sx, sy))
            if (layer, gx, gy) in node_set: goals.add((layer, gx, gy))

        if not starts or not goals:
            return self._direct_lwire(net, xa, ya, xb, yb)

        def heuristic(n):
            _, x, y = n
            return min(abs(x-gx)+abs(y-gy) for _,gx,gy in goals)

        def neighbors(n):
            layer, x, y = n
            xs, ys = candidates.get(layer, ([], []))
            out = []
            xi = xs.index(x) if x in xs else -1
            yi = ys.index(y) if y in ys else -1
            for k in (1, 2, 3):
                if xi-k >= 0:      out.append((layer, xs[xi-k], y))
                if xi+k < len(xs): out.append((layer, xs[xi+k], y))
                if yi-k >= 0:      out.append((layer, x, ys[yi-k]))
                if yi+k < len(ys): out.append((layer, x, ys[yi+k]))
            for nl in ADJ.get(layer, []):
                if nl in candidates:
                    sx, sy = self.nearest_track(nl, x, y)
                    nb = (nl, sx, sy)
                    if nb in node_set: out.append(nb)
            return out

        def move_shapes(cur, nb):
            layer = cur[0]
            if layer == nb[0]:
                if cur[1]!=nb[1]: return [(layer, hwire(layer, cur[1], nb[1], cur[2]))]
                return [(layer, vwire(layer, cur[1], cur[2], nb[2]))]
            return [(cur[0], box(cur[0], cur[1], cur[2])),
                    (nb[0],  box(nb[0],  nb[1],  nb[2]))]

        pq = []; dist = {}; parent = {}
        for st in starts[:min(4, len(starts))]:
            dist[st] = 0; heapq.heappush(pq, (heuristic(st), 0, st))

        found = None; expanded = 0
        limit = 120000 if len(self.nets) < LARGE_DESIGN_NETS else 35000

        while pq and expanded < limit:
            _, g, cur = heapq.heappop(pq)
            if g != dist.get(cur): continue
            expanded += 1
            if cur in goals: found = cur; break
            for nb in neighbors(cur):
                sh  = move_shapes(cur, nb)
                bad = self.count_bad(net, sh)
                if cur[0] == nb[0]:
                    step = manhattan_xy((cur[1],cur[2]),(nb[1],nb[2]))
                    if cur[1]!=nb[1] and DIR.get(cur[0])=="V": step *= COST_NONPREF
                    if cur[2]!=nb[2] and DIR.get(cur[0])=="H": step *= COST_NONPREF
                    cost = step + COST_SPACING*bad
                else:
                    cost = 1500 + COST_SPACING*bad
                ng = g + cost
                if ng < dist.get(nb, 10**30):
                    dist[nb] = ng; parent[nb] = cur
                    heapq.heappush(pq, (ng+heuristic(nb), ng, nb))

        if found is None:
            return (self._via_stack(xa, ya, la, allowed[0]) +
                    self._direct_lwire(net, xa, ya, xb, yb) +
                    self._via_stack(xb, yb, lb, allowed[0]))

        path = [found]
        while path[-1] in parent: path.append(parent[path[-1]])
        path.reverse()

        shapes = []
        shapes.extend(self._via_stack(xa, ya, la, path[0][0]))
        shapes.extend(self._track_connector(path[0][0], xa, ya))
        shapes.extend(self._path_to_shapes(path))
        shapes.extend(self._track_connector(path[-1][0], xb, yb))
        shapes.extend(self._via_stack(xb, yb, lb, path[-1][0]))
        return shapes

    # ── Connectivity repair loop ──────────────────────────────────────

    def repair_connectivity(self, net, shapes, max_iter=60):
        """Iterative A*-based repair until single component or no progress."""
        repaired = list(shapes) if shapes else []
        stagnant = 0

        for _ in range(max_iter):
            comps = self._net_components(net, repaired)
            if len(comps) <= 1: return repaired, 0

            base   = max(comps, key=len)
            others = [c for c in comps if c is not base]

            def comp_dist(c):
                ia, ib = self._closest_pair(c, base)
                if ia is None: return 10**9
                return manhattan_xy(ctr(ia[1]), ctr(ib[1]))

            before = len(comps); best_trial = None

            for target in sorted(others, key=comp_dist)[:min(6, len(others))]:
                bridge = self._connect_components(base, target, net)
                if not bridge: continue
                trial = repaired + bridge
                after = len(self._net_components(net, trial))
                bad   = self.count_bad(net, bridge)
                score = (after, bad, len(bridge))
                if best_trial is None or score < best_trial[0]:
                    best_trial = (score, trial)
                if after < before: break

            if best_trial is None: break
            repaired = best_trial[1]
            after    = best_trial[0][0]
            if after >= before:
                stagnant += 1
                if stagnant >= 3: break
            else:
                stagnant = 0

        return repaired, len(self._net_components(net, repaired)) - 1

    # ── Commit route to occupancy ─────────────────────────────────────

    def _add_route(self, net, shapes):
        """
        Deduplicate and commit shapes to occupancy.
        dedup prevents double-entries in checker's rtree which could
        cause spurious violation counts.
        """
        seen = set()
        for layer, r in shapes:
            if layer not in LAYERS: continue
            r = legal(layer, r)
            key = (layer,) + r
            if key in seen: continue
            seen.add(key)
            self.routes[net].append((layer, r))
            self._add_occ(layer, net, r)

    # ── Avoid-layer variant sets (index-based, no hardcoded names) ────

    def _avoid_set_from_index(self, idx):
        avoid = set()
        L = LAYERS
        # idx=0: no avoidance (use all layers)
        # idx=1: avoid lowest (li1) — reduces cell-OBS conflicts
        # idx=2: avoid lowest + layer[2]
        # idx=3: avoid lowest + layer[3]
        # idx=4: avoid lowest + layer[2] + layer[3]
        if idx >= 1 and len(L) > 0: avoid.add(L[0])
        if idx == 2 and len(L) > 2: avoid.add(L[2])
        if idx == 3 and len(L) > 3: avoid.add(L[3])
        if idx == 4 and len(L) > 2: avoid.add(L[2])
        if idx == 4 and len(L) > 3: avoid.add(L[3])
        return avoid

    # ── Single-net routing entry point ────────────────────────────────

    def route_net(self, net):
        t0 = time.time()
        groups = self.nets[net]
        if len(groups) < 2: return

        avoid_indices = [1, 0, 2, 3, 4] if len(self.nets) < LARGE_DESIGN_NETS else [1, 0, 2]
        # try li1-avoided variant FIRST (idx=1) since li1 is the primary source
        # of spacing violations in dense SKY130 designs. Fall back to full-layer if needed.

        candidates = []
        for ai in avoid_indices:
            avoid = self._avoid_set_from_index(ai)
            raw   = self._route_net_astar(net, groups, avoid_layers=avoid)
            if not raw: continue
            before = len(raw)
            shapes, remaining = self.repair_connectivity(net, raw)
            repaired = len(shapes) - before
            conflict = self.count_bad(net, shapes) if shapes else 10**9
            metric   = (remaining, conflict, repaired, len(shapes))
            candidates.append((metric, shapes, repaired, remaining, conflict, avoid))
            if remaining == 0 and conflict == 0: break

        if not candidates:
            print(f"[WARN] {net}: A* failed all variants — using direct L-wire fallback")
            all_pts = []
            for _, _, lr in groups:
                for rects in lr.values():
                    for r in rects: all_pts.append(ctr(r))
            if len(all_pts) >= 2:
                edges  = rmst_edges(all_pts)
                shapes = []
                best_layer = self._preferred_layers()[0]
                for pt in all_pts:
                    shapes.append((best_layer, box(best_layer, pt[0], pt[1])))
                for a, b in edges:
                    shapes.extend(self._direct_lwire(net, a[0], a[1], b[0], b[1]))
                shapes, remaining = self.repair_connectivity(net, shapes)
                conflict = self.count_bad(net, shapes)
                self._add_route(net, shapes)
                print(f"[FALLBACK] {net}: shapes={len(shapes)}, opens={remaining}, drc={conflict}, time={time.time()-t0:.3f}s")
            return

        metric, shapes, repaired, remaining, conflict, avoid = min(candidates, key=lambda x: x[0])
        self._add_route(net, shapes)
        tag = "A*" if repaired == 0 else "A*+REPAIR"
        print(f"[{tag}] {net}: shapes={len(shapes)}, repair={repaired}, opens={remaining}, drc={conflict}, time={time.time()-t0:.3f}s")

    # ── Net ordering ──────────────────────────────────────────────────

    def _net_score(self, net):
        pts = []
        for _, _, lr in self.nets[net]:
            for rects in lr.values():
                for r in rects: pts.append(ctr(r))
        if not pts: return 0
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        return len(pts)*100000 + (max(xs)-min(xs)) + (max(ys)-min(ys))

    # ── Route all nets ────────────────────────────────────────────────

    def route_all(self):
        order = sorted(self.nets, key=self._net_score, reverse=True)
        start = time.time()
        for idx, net in enumerate(order, 1):
            self.route_net(net)
            if idx % 100 == 0:
                print(f"[PROGRESS] {idx}/{len(order)} nets in {time.time()-start:.1f}s")
        print(f"[DONE] Routed {len(order)} nets in {time.time()-start:.1f}s")
        return self.routes


# ──────────────────────────────────────────────────────────────────────
# Public entry point (required by project spec)
# ──────────────────────────────────────────────────────────────────────

def detailed_route(input_def, input_lef, input_guide, output_def):
    """
    detailed_route(input_def, input_lef, input_guide, output_def)
    Reads placement DEF, technology LEF, and global routing GUIDE;
    computes detailed routes for every signal net; writes output DEF.
    """
    die, tracks, nets, obstacles, guides = extract_design(input_def, input_lef, input_guide)
    print(f"[INFO] Design: {len(nets)} nets, die={die}")

    router = DetailedRouter(die, tracks, nets, obstacles, guides)
    routes = router.route_all()

    out_def = LEFDEFParser.DEFReader()
    out_def.readDEF(input_def)

    for net in out_def.nets():
        name = net.name()
        if name not in routes: continue
        seen = set()
        for layer, r in routes[name]:
            x1, y1, x2, y2 = legal(layer, r)
            if x2 > x1 and y2 > y1:
                key = (layer, x1, y1, x2, y2)
                if key not in seen:
                    seen.add(key)
                    net.addRect(layer, int(x1), int(y1), int(x2), int(y2))

    out_def.writeDEF(output_def)
    print(f"[OK] wrote {output_def}")


# ──────────────────────────────────────────────────────────────────────
# CLI wrapper
# ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Guide-Aware Detailed Router")
    ap.add_argument("-i", "--idef",  required=False)
    ap.add_argument("-l", "--lef",   required=False)
    ap.add_argument("-g", "--guide", required=False)
    ap.add_argument("-o", "--odef",  required=False)
    ap.add_argument("positional", nargs="*")
    args = ap.parse_args()

    if len(args.positional) == 4:
        detailed_route(*args.positional)
    elif args.idef and args.lef and args.guide and args.odef:
        detailed_route(args.idef, args.lef, args.guide, args.odef)
    else:
        print("Usage:")
        print("  python3 detailed_router.py -i input.def -l input.lef -g input.guide -o output.def")
        print("  python3 detailed_router.py input.def input.lef input.guide output.def")


if __name__ == "__main__":
    main()
