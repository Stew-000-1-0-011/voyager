"""Perfboard routing, all signal wiring on the solder side in two passes.

- The motor drivers take battery power straight from the harness, so the board only carries small
  currents (buck module input, battery sense) and every net, VBAT and GND included, is routed alike.
- Grid routing with negotiated congestion (PathFinder), in two kinds of wire:
    BARE  bare tinned wire laid along the holes, soldered at every hole it runs over;
    WIRE  insulated wire laid along the holes on top of the bare wiring, so it may cross bare traces.
          It is soldered only at its two ends: onto a pin's joint, or onto a bare trace ("via").
  A net's tree may branch anywhere on bare wire but only at the ends of an insulated wire, because an
  insulated wire cannot be tapped mid-run. Neither kind passes over another pin's solder joint.
"""

from __future__ import annotations

import heapq
import random
from dataclasses import dataclass, field

from netlist import BOARD_HOLES, MODULE_BRIDGES, PARTS, PLACEMENT, nets, pin_hole

Hole = tuple[int, int]

BARE = "bare"
WIRE = "wire"
_LAYERS = (BARE, WIRE)


STEP_COST = {BARE: 1.0, WIRE: 1.4}  # prefer bare solder-side traces
TURN_COST = 1.0
VIA_COST = 4.0
MAX_ITER = 60
_DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NC = "__NC__"
STATS: dict[str, int] = {}  # last negotiation: iterations used, holes still contested


@dataclass(frozen=True)
class Trace:
    net: str
    holes: tuple[Hole, ...]
    layer: str = BARE


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


def ground_ring() -> list[Hole]:
    """Bare GND wire around the perimeter on the solder side (pin 1 of every cable connector is on it)."""
    cols, rows = BOARD_HOLES
    path = [(c, 0) for c in range(cols)] + [(cols - 1, r) for r in range(1, rows)]
    path += [(c, rows - 1) for c in range(cols - 2, -1, -1)] + [(0, r) for r in range(rows - 2, 0, -1)]
    return path


# ---------------------------------------------------------------------------
# two-layer search

class _Board:
    def __init__(self) -> None:
        self.cols, self.rows = BOARD_HOLES
        self.pin_owner: dict[Hole, str] = {}
        for ref in ("J_BP_A", "J_BP_B"):  # unconnected Bluepill pins are still pins
            for i in range(1, 21):
                self.pin_owner[pin_hole(ref, str(i))] = _NC
        for p in PARTS:
            for pin, net in p.pins.items():
                self.pin_owner[pin_hole(p.ref, pin)] = net
        self.ring = set(ground_ring())
        # an insulated wire may cross the ring but not lie along it, on top of its solder joints
        cols, rows = self.cols, self.rows
        self.ring_axes: dict[Hole, set[str]] = {}
        for c, r in self.ring:
            axes = self.ring_axes.setdefault((c, r), set())
            if r in (0, rows - 1):
                axes.add("h")
            if c in (0, cols - 1):
                axes.add("v")

    def inside(self, h: Hole) -> bool:
        return 0 <= h[0] < self.cols and 0 <= h[1] < self.rows

    def enterable(self, net: str, layer: str, h: Hole, targets: set[Hole]) -> bool:
        """May `net` occupy hole h on `layer` (ignoring other nets' routed wires)?"""
        owner = self.pin_owner.get(h)
        if owner is not None and owner != net:
            return False
        if layer == BARE and h in self.ring and net != "GND":
            return False  # the GND ring wire runs through every perimeter hole
        if layer == WIRE:
            if owner == net and h not in targets:
                return False  # cannot run over a lead; own pins are only end points
        return True


def _axis(direction: int) -> str:
    return "h" if _DIRS[direction][1] == 0 else "v"


def _key(layer: str, h: Hole, axis: str | None = None) -> tuple:
    """Occupancy key: a bare-wire hole is used whole; an insulated wire per axis, so insulated wires may cross."""
    return ("B", h) if layer == BARE else ("T", h, axis)


def _grow(board: _Board, net: str, roots: set[Hole], pins: set[Hole], cost_of):
    """Connect all pins to the roots; returns (segments [(layer, holes)], unreached pins).
    cost_of(key) prices an occupancy key (see _key); inf means forbidden."""
    inf = float("inf")
    bottom_tree: set[Hole] = set(roots)
    entries: set[Hole] = set(roots)  # holes where an insulated wire may start or end
    remaining = set(pins) - entries
    segments: list[tuple[str, list[Hole]]] = []
    while remaining:
        best: dict[tuple[str, Hole, int], float] = {}
        prev: dict[tuple[str, Hole, int], tuple[str, Hole, int] | None] = {}
        heap: list[tuple[float, int, str, Hole, int]] = []
        tie = 0
        for layer, holes in ((BARE, bottom_tree), (WIRE, entries)):
            for h in holes:
                best[(layer, h, -1)] = 0.0
                prev[(layer, h, -1)] = None
                heap.append((0.0, tie, layer, h, -1))
                tie += 1
        heapq.heapify(heap)
        found = None
        while heap:
            cost, _, layer, h, d = heapq.heappop(heap)
            if cost > best[(layer, h, d)]:
                continue
            if h in remaining and prev[(layer, h, d)] is not None:
                found = (layer, h, d)
                break
            # switch between bare and insulated wire at this hole (a via takes the hole for both kinds)
            other = WIRE if layer == BARE else BARE
            if (d != -1 and board.pin_owner.get(h) is None
                    and board.enterable(net, other, h, remaining)):
                if other == WIRE:
                    step = cost_of(_key(WIRE, h, "h")) + cost_of(_key(WIRE, h, "v"))
                else:
                    step = cost_of(_key(BARE, h)) + cost_of(_key(WIRE, h, "h" if _axis(d) == "v" else "v"))
                if step < inf:
                    ncost = cost + VIA_COST + step
                    key = (other, h, -1)
                    if ncost < best.get(key, inf):
                        best[key] = ncost
                        prev[key] = (layer, h, d)
                        tie += 1
                        heapq.heappush(heap, (ncost, tie, other, h, -1))
            if layer == WIRE and d != -1 and board.pin_owner.get(h) == net:
                continue  # an insulated wire that reached one of this net's pins ends there
            for nd, (dc, dr) in enumerate(_DIRS):
                n = (h[0] + dc, h[1] + dr)
                if not board.inside(n) or not board.enterable(net, layer, n, remaining):
                    continue
                if layer == WIRE and (_axis(nd) in board.ring_axes.get(n, ())
                                      or _axis(nd) in board.ring_axes.get(h, ())):
                    continue
                step = cost_of(_key(layer, n, _axis(nd)))
                turning = d not in (-1, nd)
                if layer == WIRE and turning:
                    step += cost_of(_key(WIRE, h, _axis(nd)))  # a bend uses both axes of its hole
                if step == inf:
                    continue
                ncost = cost + STEP_COST[layer] + step + (TURN_COST if turning else 0)
                key = (layer, n, nd)
                if ncost < best.get(key, inf):
                    best[key] = ncost
                    prev[key] = (layer, h, d)
                    tie += 1
                    heapq.heappush(heap, (ncost, tie, layer, n, nd))
        if found is None:
            break
        path: list[tuple[str, Hole]] = []
        state = found
        while state is not None:
            path.append((state[0], state[1]))
            state = prev[state]
        path.reverse()
        # split into single-layer runs; a layer change happens in place at a hole (via)
        run_layer, run = path[0][0], [path[0][1]]
        new_segments = []
        for layer, h in path[1:]:
            if layer != run_layer:
                if len(run) > 1:
                    new_segments.append((run_layer, run))
                entries.add(h)
                bottom_tree.add(h)  # the via is soldered on the bottom side
                run_layer, run = layer, [h]
            else:
                run.append(h)
        if len(run) > 1:
            new_segments.append((run_layer, run))
        for layer, holes in new_segments:
            if layer == BARE:
                bottom_tree.update(holes)
            else:
                entries.update((holes[0], holes[-1]))
        segments += new_segments
        entries.add(found[1])
        bottom_tree.add(found[1])
        remaining -= bottom_tree | entries
    return segments, remaining


def _usage(segments, pins: set[Hole]) -> set[tuple]:
    used: set[tuple] = set()
    for layer, holes in segments:
        for i, h in enumerate(holes):
            if h in pins:
                continue
            if layer == BARE:
                used.add(_key(BARE, h))
                continue
            if i in (0, len(holes) - 1):  # end of an insulated wire = via: the whole hole
                used.update({_key(WIRE, h, "h"), _key(WIRE, h, "v"), _key(BARE, h)})
                continue
            for j in (i - 1, i + 1):
                used.add(_key(WIRE, h, "h" if holes[j][1] == h[1] else "v"))
    return used


def route() -> Routing:
    board = _Board()
    all_nets = nets()
    pins = {n: {pin_hole(ref, pin) for ref, pin in conns} for n, conns in all_nets.items()}
    bridged = {n: {pin_hole(ref, pin) for ref, pin in g} for n, g in MODULE_BRIDGES.items()}
    if "GND" in bridged:
        # the ring is the root: one Blue Pill ground pin wired to it brings the module's others along
        keep = max(bridged["GND"], key=lambda h: h[1])
        pins["GND"] -= bridged["GND"] - {keep}
    order = sorted(all_nets, key=lambda n: (len(all_nets[n]), n))

    ring = ground_ring()
    fixed = {"GND": [(BARE, ring)]}

    def roots(net: str) -> set[Hole]:
        out = {min(pins[net])}
        if net in bridged and net != "GND":
            out = set(bridged[net])
        if net == "GND":
            out |= set(ring)
        return out

    history: dict[tuple, float] = {}
    usage: dict[str, set[tuple]] = {n: set() for n in order}
    routed: dict[str, list[tuple[str, list[Hole]]]] = {}
    rng = random.Random(1)
    for it in range(MAX_ITER):
        present = 0.5 * (1.5 ** min(it, 12))
        sweep = list(order)
        if it:
            rng.shuffle(sweep)  # break ties between nets that keep fighting for the same holes
        for net in sweep:
            usage[net] = set()
            occ: dict[tuple, int] = {}
            for cells in usage.values():
                for c in cells:
                    occ[c] = occ.get(c, 0) + 1

            def cost_of(key: tuple, occ=occ) -> float:
                return history.get(key, 0.0) + present * occ.get(key, 0)

            segs, _ = _grow(board, net, roots(net), pins[net], cost_of)
            segs = fixed.get(net, []) + segs
            routed[net] = segs
            usage[net] = _usage(segs, pins[net])
        count: dict[tuple, int] = {}
        for cells in usage.values():
            for c in cells:
                count[c] = count.get(c, 0) + 1
        overused = [c for c, k in count.items() if k > 1]
        STATS["iterations"], STATS["overused"] = it + 1, len(overused)
        if not overused:
            break
        for c in overused:
            history[c] = history.get(c, 0.0) + 1.0 + 0.5 * it

    # commit: nets keep their negotiated route when it is still free, otherwise they are rerouted with the
    # already committed wires as walls; nets that still fail are moved to the front and the commit retried
    def commit(seq: list[str]) -> tuple[Routing, list[str]]:
        res = Routing()
        taken: dict[tuple, str] = {}
        failed: list[str] = []
        for net in seq:
            if all(taken.get(c, net) == net for c in usage[net]):
                segs, left = routed[net], set()
            else:
                def hard(key: tuple, net=net) -> float:
                    # committed wires are walls (insulated wires per axis, so they may still be crossed)
                    return 0.0 if taken.get(key, net) == net else float("inf")

                segs, left = _grow(board, net, roots(net), pins[net], hard)
                segs = fixed.get(net, []) + segs
            for c in _usage(segs, pins[net]):
                taken[c] = net
            res.traces += [Trace(net, tuple(h), layer) for layer, h in segs]
            tree = roots(net) | {h for _, hs in segs for h in hs}
            for b in sorted(left):
                a = min(tree, key=lambda t: abs(t[0] - b[0]) + abs(t[1] - b[1]))
                res.jumpers.append(Jumper(net, a, b, "AWG26"))
                tree.add(b)
            if left:
                failed.append(net)
        return res, failed

    seq = sorted(order, key=lambda n: (len(usage[n]), n))
    best, failed = commit(seq)
    for attempt in range(20):
        if not failed:
            break
        # failed nets go first; the rest keep their order, lightly shuffled after a few tries
        rest = [n for n in seq if n not in failed]
        if attempt >= 3:
            rng.shuffle(rest)
        seq = failed + rest
        cand, failed = commit(seq)
        if len(cand.jumpers) < len(best.jumpers):
            best = cand
    STATS["fallback_wires"] = len(best.jumpers)
    return best


__all__ = ["BARE", "WIRE", "Trace", "Jumper", "Routing", "route", "PLACEMENT"]
