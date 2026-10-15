from netlist import BOARD_HOLES, PARTS, nets, pin_hole
from route import HEAVY_NETS, route

R = route()


def test_every_net_fully_connected():
    for net, conns in nets().items():
        holes = {pin_hole(ref, pin) for ref, pin in conns}
        parent = {h: h for h in holes}

        def find(x):
            while parent.setdefault(x, x) != x:
                x = parent[x]
            return x

        def union(a, b):
            parent[find(a)] = find(b)

        for t in R.traces:
            if t.net == net:
                for a, b in zip(t.holes, t.holes[1:]):
                    union(a, b)
        for j in R.jumpers:
            if j.net == net:
                union(j.a, j.b)
        assert len({find(h) for h in holes}) == 1, net


def test_traces_step_to_adjacent_holes_only():
    for t in R.traces:
        for (c1, r1), (c2, r2) in zip(t.holes, t.holes[1:]):
            assert abs(c1 - c2) + abs(r1 - r2) == 1, t.net


def test_bottom_holes_not_shared_between_nets():
    owner = {}
    for p in PARTS:
        for pin, net in p.pins.items():
            owner[pin_hole(p.ref, pin)] = net
    for t in R.traces:
        for h in t.holes:
            assert owner.setdefault(h, t.net) == t.net, (h, t.net, owner[h])


def test_traces_inside_board():
    for t in R.traces:
        for c, r in t.holes:
            assert 0 <= c < BOARD_HOLES[0] and 0 <= r < BOARD_HOLES[1]


def test_power_star_uses_heavy_wire():
    heavy = [j for j in R.jumpers if j.gauge == "AWG18"]
    assert {j.net for j in heavy} == set(HEAVY_NETS)
    vm_refs = {"J_VM_L", "J_VM_R", "J_VM_G", "J_BUCK"}
    ends = {(j.net, j.b) for j in heavy}
    for p in PARTS:
        if p.ref in vm_refs:
            for pin, net in p.pins.items():
                if net in HEAVY_NETS:
                    assert (net, pin_hole(p.ref, pin)) in ends, (p.ref, pin)


def test_insulated_wires_bounded():
    # Two 20-pin Bluepill headers fanning out to all four edges is not planar on one layer,
    # so nets that cannot stay on bare solder-side traces use insulated wires.
    assert len([j for j in R.jumpers if j.gauge == "AWG26"]) <= 40


def test_most_nets_use_bare_traces():
    bare = {t.net for t in R.traces}
    assert len(bare) >= len(nets()) // 2


def test_traces_avoid_unconnected_pins():
    # Bluepill NC pins (VB, PC13-15, R, PA11) are still physical pins: no trace may touch them
    nc = {pin_hole(ref, str(i)) for ref in ("J_BP_A", "J_BP_B") for i in range(1, 21)}
    nc -= {pin_hole(p.ref, pin) for p in PARTS for pin in p.pins}
    assert nc
    for t in R.traces:
        assert not nc & set(t.holes), (t.net, nc & set(t.holes))
