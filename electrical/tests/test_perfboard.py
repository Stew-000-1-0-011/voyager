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
        if p.ref.startswith(("J_ENC", "J_US", "J_DRV", "J_PWR", "J_BUCK")):
            holes = [pin_hole(p.ref, pin) for pin in p.pins]
            assert any(c in (0, BOARD_HOLES[0] - 1) or r in (0, BOARD_HOLES[1] - 1) for c, r in holes), p.ref


def test_cable_connectors_have_ground_on_the_board_edge():
    # the GND pin (first or last, to match the module's header) sits on the perimeter, where one bare ring
    # wire picks them all up; every other pin is inside
    from netlist import ground_pin

    cols, rows = BOARD_HOLES
    for p in PARTS:
        if p.ref.startswith(("J_ENC", "J_US", "J_DRV", "J_PWR", "J_BUCK", "J_UART")):
            g = ground_pin(p)
            assert g is not None, p.ref
            c, r = pin_hole(p.ref, g)
            assert c in (0, cols - 1) or r in (0, rows - 1), p.ref
            others = [pin_hole(p.ref, pin) for pin in p.pins if pin != g]
            assert all(0 < c < cols - 1 and 0 < r < rows - 1 for c, r in others), p.ref


def test_cables_run_straight_to_the_modules():
    by_ref = {p.ref: p for p in PARTS}
    assert by_ref["J_DRV_L"].pins == {"1": "GND", "3": "L_IN1", "4": "L_IN2"}  # GND, VM, IN1, IN2
    assert [by_ref["J_US_LF"].pins[k] for k in "1234"] == ["+5V", "US_TRIG_LF", "US_ECHO_LF_RAW", "GND"]


def test_part_outlines_do_not_overlap():
    from netlist import part_outline

    seen: dict[tuple[int, int], str] = {}
    for p in PARTS:
        for h in part_outline(p.ref):
            assert seen.setdefault(h, p.ref) == p.ref, (p.ref, seen[h], h)
