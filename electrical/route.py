"""Perfboard routing.

- VBAT / GND power: star of AWG18 wires on the component side from J_PWR.
- Everything else: solder-side traces found by a turn-penalised grid search
  (Dijkstra over (hole, direction)), grown as a tree per net.
- Pin pairs that cannot be routed on the solder side become AWG26 jumpers.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from netlist import BOARD_HOLES, PARTS, nets, pin_hole

Hole = tuple[int, int]

HEAVY_NETS = ("VBAT", "GND")
POWER_ROOT = "J_PWR"
STAR_REFS = ("J_VM_L", "J_VM_G", "J_VM_R", "J_BUCK", "C_BULK")
STEP_COST = 1
TURN_COST = 1
_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))


@dataclass(frozen=True)
class Trace:
    net: str
    holes: tuple[Hole, ...]


@dataclass(frozen=True)
class Jumper:
    net: str
    a: Hole
    b: Hole
    gauge: str


@dataclass
class Routing:
    traces: list[Trace] = field(default_factory=list)
    jumpers: list[Jumper] = field(default_factory=list)


def _power_star() -> list[Jumper]:
    root = {net: pin_hole(POWER_ROOT, pin) for pin, net in
            next(p for p in PARTS if p.ref == POWER_ROOT).pins.items()}
    out = []
    for p in PARTS:
        if p.ref in STAR_REFS:
            for pin, net in p.pins.items():
                if net in HEAVY_NETS:
                    out.append(Jumper(net, root[net], pin_hole(p.ref, pin), "AWG18"))
    return out


def _grow_tree(net: str, tree: set[Hole], targets: set[Hole], blocked: dict[Hole, str],
               cost_of) -> tuple[list[list[Hole]], set[Hole]]:
    """Connect every target to the tree; returns (paths, unreached targets)."""
    cols, rows = BOARD_HOLES
    tree = set(tree)
    remaining = set(targets) - tree
    paths: list[list[Hole]] = []
    while remaining:
        best: dict[tuple[Hole, int], float] = {}
        prev: dict[tuple[Hole, int], tuple[Hole, int] | None] = {}
        heap: list[tuple[float, Hole, int]] = []
        for h in tree:
            best[(h, -1)] = 0.0
            prev[(h, -1)] = None
            heap.append((0.0, h, -1))
        heapq.heapify(heap)
        found = None
        while heap:
            cost, h, d = heapq.heappop(heap)
            if cost > best[(h, d)]:
                continue
            if h in remaining:
                found = (h, d)
                break
            for nd, (dc, dr) in enumerate(_DIRS):
                n = (h[0] + dc, h[1] + dr)
                if not (0 <= n[0] < cols and 0 <= n[1] < rows):
                    continue
                if blocked.get(n, net) != net:
                    continue
                if blocked.get(n) == net and n not in tree and n not in remaining:
                    continue  # this net's own pins are entered only as targets
                ncost = cost + cost_of(n) + (TURN_COST if d not in (-1, nd) else 0)
                if ncost < best.get((n, nd), float("inf")):
                    best[(n, nd)] = ncost
                    prev[(n, nd)] = (h, d)
                    heapq.heappush(heap, (ncost, n, nd))
        if found is None:
            break
        path, state = [], found
        while state is not None:
            path.append(state[0])
            state = prev[state]
        path.reverse()
        paths.append(path)
        tree.update(path)
        remaining -= set(path)
    return paths, remaining


def _roots(net: str, pins: set[Hole], star: list[Jumper]) -> set[Hole]:
    if net in HEAVY_NETS:
        root = next(pin_hole(POWER_ROOT, pin) for pin, n in
                    next(p for p in PARTS if p.ref == POWER_ROOT).pins.items() if n == net)
        return {root} | {j.b for j in star if j.net == net}
    return {min(pins)}


MAX_ITER = 15  # overuse plateaus well before this (single-layer routing is non-planar here)


def route() -> Routing:
    star = _power_star()
    pin_owner: dict[Hole, str] = {}
    for ref in ("J_BP_A", "J_BP_B"):  # unconnected Bluepill pins are still pins
        for i in range(1, 21):
            pin_owner[pin_hole(ref, str(i))] = "__NC__"
    for p in PARTS:
        for pin, net in p.pins.items():
            pin_owner[pin_hole(p.ref, pin)] = net
    all_nets = nets()
    order = sorted(all_nets, key=lambda n: (len(all_nets[n]), n))
    pins = {n: {pin_hole(ref, pin) for ref, pin in all_nets[n]} for n in order}

    # negotiated congestion: nets may share holes at a rising price until nobody does
    history: dict[Hole, float] = {}
    usage: dict[str, set[Hole]] = {n: set() for n in order}
    routed: dict[str, list[list[Hole]]] = {}
    for it in range(MAX_ITER):
        present = 0.5 * (1.6 ** it)
        for net in order:
            usage[net] = set()
            occ: dict[Hole, int] = {}
            for other, cells in usage.items():
                for c in cells:
                    occ[c] = occ.get(c, 0) + 1

            def cost_of(h: Hole, occ=occ) -> float:
                return STEP_COST + history.get(h, 0.0) + present * occ.get(h, 0)

            paths, _ = _grow_tree(net, _roots(net, pins[net], star), pins[net], pin_owner, cost_of)
            routed[net] = paths
            usage[net] = {h for path in paths for h in path} - pins[net]
        count: dict[Hole, int] = {}
        for cells in usage.values():
            for c in cells:
                count[c] = count.get(c, 0) + 1
        overused = [c for c, k in count.items() if k > 1]
        if not overused:
            break
        for c in overused:
            history[c] = history.get(c, 0.0) + 1.0

    # commit: legal nets keep their routes; any remaining conflict is rerouted hard-blocked, then jumpered
    result = Routing(jumpers=list(star))
    owner = dict(pin_owner)
    pending = []
    # keep the shortest negotiated routes first: they block the fewest others
    for net in sorted(order, key=lambda n: (len(usage[n]), n)):
        cells = usage[net]
        if any(owner.get(c, net) != net for c in cells):
            pending.append(net)
            continue
        for c in cells:
            owner[c] = net
        result.traces += [Trace(net, tuple(p)) for p in routed[net]]
    for net in pending:
        paths, left = _grow_tree(net, _roots(net, pins[net], star), pins[net], owner, lambda h: STEP_COST)
        for p in paths:
            for c in p:
                owner[c] = net
        result.traces += [Trace(net, tuple(p)) for p in paths]
        tree = _roots(net, pins[net], star) | {c for p in paths for c in p}
        for b in sorted(left):
            a = min(tree, key=lambda t: abs(t[0] - b[0]) + abs(t[1] - b[1]))
            result.jumpers.append(Jumper(net, a, b, "AWG26"))
            tree.add(b)
    return result
