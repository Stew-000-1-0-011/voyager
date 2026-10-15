from netlist import BOARD_HOLES, PARTS, PLACEMENT, pin_hole


def test_every_part_placed_on_board():
    assert set(PLACEMENT) == {p.ref for p in PARTS}
    for p in PARTS:
        for pin in p.pins:
            c, r = pin_hole(p.ref, pin)
            assert 0 <= c < BOARD_HOLES[0] and 0 <= r < BOARD_HOLES[1], (p.ref, pin)


def test_no_two_pins_share_a_hole():
    holes = [pin_hole(p.ref, pin) for p in PARTS for pin in p.pins]
    assert len(holes) == len(set(holes))


def test_bluepill_rows_are_15_24mm_apart():
    a1, b1 = pin_hole("J_BP_A", "1"), pin_hole("J_BP_B", "1")
    assert abs(a1[0] - b1[0]) + abs(a1[1] - b1[1]) == 6  # 6 holes = 15.24mm


def test_cable_connectors_on_board_edge():
    for p in PARTS:
        if p.ref.startswith(("J_ENC", "J_US", "J_DRV", "J_VM", "J_PWR", "J_BUCK")):
            holes = [pin_hole(p.ref, pin) for pin in p.pins]
            assert any(c in (0, BOARD_HOLES[0] - 1) or r in (0, BOARD_HOLES[1] - 1) for c, r in holes), p.ref
