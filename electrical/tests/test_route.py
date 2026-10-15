from collections import Counter

from netlist import BOARD_HOLES, MODULE_BRIDGES, PARTS, nets, pin_hole
from route import BOTTOM, HEAVY_NETS, TOP, route, top_keepout

R = route()
PIN_OWNER = {pin_hole(p.ref, pin): net for p in PARTS for pin, net in p.pins.items()}
BLUEPILL_HOLES = {pin_hole(ref, str(i)) for ref in ("J_BP_A", "J_BP_B") for i in range(1, 21)}


def _find(parent, x):
    while parent.setdefault(x, x) != x:
        x = parent[x]
    return x


def test_every_net_fully_connected():
    for net, conns in nets().items():
        holes = {pin_hole(ref, pin) for ref, pin in conns}
        parent = {}
        group = [pin_hole(ref, pin) for ref, pin in MODULE_BRIDGES.get(net, [])]
        for a, b in zip(group, group[1:]):
            parent[_find(parent, a)] = _find(parent, b)  # tied together on the Blue Pill itself
        for t in R.traces:
            if t.net != net:
                continue
            if t.layer == BOTTOM:
                # a bare trace is soldered in every hole it runs through
                for a, b in zip(t.holes, t.holes[1:]):
                    parent[_find(parent, a)] = _find(parent, b)
            else:
                # an insulated wire only connects where it enters a hole: its two ends
                parent[_find(parent, t.holes[0])] = _find(parent, t.holes[-1])
        for j in R.jumpers:
            if j.net == net:
                parent[_find(parent, j.a)] = _find(parent, j.b)
        assert len({_find(parent, h) for h in holes}) == 1, net


def test_traces_step_to_adjacent_holes_only():
    for t in R.traces:
        assert len(t.holes) >= 2
        for (c1, r1), (c2, r2) in zip(t.holes, t.holes[1:]):
            assert abs(c1 - c2) + abs(r1 - r2) == 1, t.net


def test_traces_inside_board():
    for t in R.traces:
        for c, r in t.holes:
            assert 0 <= c < BOARD_HOLES[0] and 0 <= r < BOARD_HOLES[1]


def test_bottom_holes_not_shared_between_nets():
    owner = dict(PIN_OWNER)
    for t in R.traces:
        if t.layer == BOTTOM:
            for h in t.holes:
                assert owner.setdefault(h, t.net) == t.net, (h, t.net, owner[h])


def _top_axes(t):
    """(hole, axis) pairs a top wire occupies: 'h'/'v' where it runs straight, both at bends and ends."""
    out = []
    for i, h in enumerate(t.holes):
        axes = set()
        for j in (i - 1, i + 1):
            if 0 <= j < len(t.holes):
                axes.add("h" if t.holes[j][1] == h[1] else "v")
        if i in (0, len(t.holes) - 1):
            axes = {"h", "v"}
        out += [(h, a) for a in axes]
    return out


def test_top_wires_only_cross_never_overlap():
    # insulated wires may cross at right angles, but never run along each other or share an end hole
    owner = {}
    for t in R.traces:
        if t.layer == TOP:
            for key in _top_axes(t):
                assert owner.setdefault(key, t.net) == t.net, (key, t.net, owner[key])


def test_top_wires_never_end_in_a_pin_hole():
    # a pin's hole is already filled by its lead or socket pin; the wire goes into a free hole beside it
    for t in R.traces:
        if t.layer == TOP:
            assert t.holes[0] not in PIN_OWNER and t.holes[-1] not in PIN_OWNER, t.net
            assert t.holes[0] not in BLUEPILL_HOLES and t.holes[-1] not in BLUEPILL_HOLES, t.net


def test_top_wires_enter_holes_only_at_their_ends():
    # a wire may not pass over a lead sticking up through the board, nor under a component body
    blocked = top_keepout()
    for t in R.traces:
        if t.layer == TOP:
            for h in t.holes[1:-1]:
                assert h not in PIN_OWNER and h not in blocked, (t.net, h)


def test_traces_avoid_unconnected_pins():
    nc = BLUEPILL_HOLES - set(PIN_OWNER)
    assert nc
    for t in R.traces:
        assert not nc & set(t.holes), (t.net, nc & set(t.holes))


def test_no_point_to_point_signal_wires():
    assert [j for j in R.jumpers if j.gauge != "AWG18"] == []


def test_power_wires_reach_every_power_pin():
    heavy_ends = {(j.net, h) for j in R.jumpers if j.gauge == "AWG18" for h in (j.a, j.b)}
    for p in PARTS:
        if p.ref in ("J_PWR", "J_BUCK", "C_BULK"):
            for pin, net in p.pins.items():
                if net in HEAVY_NETS:
                    assert (net, pin_hole(p.ref, pin)) in heavy_ends, (p.ref, pin)


def test_power_wires_daisy_chained():
    per_hole = Counter(h for j in R.jumpers if j.gauge == "AWG18" for h in (j.a, j.b))
    assert max(per_hole.values()) <= 3


def test_ground_ring_runs_around_the_perimeter():
    cols, rows = BOARD_HOLES
    perimeter = {(c, r) for c in range(cols) for r in range(rows) if c in (0, cols - 1) or r in (0, rows - 1)}
    ring = {h for t in R.traces if t.net == "GND" and t.layer == BOTTOM for h in t.holes}
    assert perimeter <= ring


def test_bus_layout_keeps_wiring_short():
    # 45 insulated wires before the GND ring, 34 with it; 6 once pins and connectors were grouped by side
    assert len([t for t in R.traces if t.layer == TOP]) <= 8
