from netlist import FT_PINS, MCU_PIN_NET, PARTS, nets


def test_spec_pin_assignment():
    expected = {
        "PB6": "L_IN1", "PB7": "L_IN2", "PB8": "R_IN1", "PB9": "R_IN2",
        "PA8": "G_IN1", "PA9": "G_IN2",
        "PA0": "ENC_L_A", "PA1": "ENC_L_B", "PA6": "ENC_R_A", "PA7": "ENC_R_B",
        "PB3": "ENC_G_A", "PB4": "ENC_G_B",
        "PB0": "US_TRIG_F", "PB1": "US_TRIG_R",
        "PB12": "US_ECHO_LF", "PB13": "US_ECHO_RF", "PB14": "US_ECHO_LR", "PB15": "US_ECHO_RR",
        "PA4": "VBAT_SENSE", "PA2": "UART_TX", "PA3": "UART_RX",
        "PB10": "I2C_SCL", "PB11": "I2C_SDA", "PA12": "USB_DP",
        "PA5": "SPARE_PA5", "PA10": "SPARE_PA10", "PA15": "SPARE_PA15", "PB5": "SPARE_PB5",
        "5V": "+5V_BP", "3V3": "+3V3", "G": "GND",
    }
    for pin, net in expected.items():
        assert MCU_PIN_NET[pin] == net, pin


def test_no_single_pin_nets():
    for net, conns in nets().items():
        assert len(conns) >= 2, net


def test_mcu_pins_unique():
    signal = {p: n for p, n in MCU_PIN_NET.items() if n not in ("GND", "+3V3")}
    assert len(set(signal.values())) == len(signal)


def test_5v_signals_only_on_ft_pins():
    # ECHO は 1kΩ 経由。抵抗の MCU 側ネットが FT ピンにだけつながること
    for name in ("US_ECHO_LF", "US_ECHO_RF", "US_ECHO_LR", "US_ECHO_RR"):
        mcu = [p for p, n in MCU_PIN_NET.items() if n == name]
        assert mcu and all(p in FT_PINS for p in mcu)
    for p in PARTS:
        if p.ref.startswith("J_ENC"):
            assert "+3V3" in p.pins.values()
            assert "+5V" not in p.pins.values()


def test_divider_and_diode():
    by_ref = {p.ref: p for p in PARTS}
    assert set(by_ref["R_DIV_H"].pins.values()) == {"VBAT", "VBAT_SENSE"}
    assert set(by_ref["R_DIV_L"].pins.values()) == {"VBAT_SENSE", "GND"}
    assert by_ref["D_5V"].pins == {"1": "+5V_BP", "2": "+5V"}  # 1=K, 2=A


def test_bluepill_rows_face_each_other_correctly():
    # Standard Blue Pill: across the board PB12 faces GND at the USB end and 3.3V faces VB at the far end.
    from netlist import BLUEPILL_ROW_A, BLUEPILL_ROW_B

    pairs = set(zip(BLUEPILL_ROW_A, BLUEPILL_ROW_B))
    for pair in (("3V3", "VB"), ("G", "PC13"), ("5V", "PC14"), ("PB8", "PA0"), ("PA8", "PB11"), ("PB12", "G")):
        assert pair in pairs, pair
