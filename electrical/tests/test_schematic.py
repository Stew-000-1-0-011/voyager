import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from netlist import nets

ROOT = Path(__file__).resolve().parents[1]


def run_erc():
    return subprocess.run(["bash", str(ROOT / "scripts/erc.sh")], capture_output=True, text=True)


def test_erc_has_no_errors():
    r = run_erc()
    assert r.returncode == 0, r.stdout + r.stderr


def test_schematic_netlist_matches_definition():
    run_erc()
    tree = ET.parse(ROOT / "build/carrier.net")
    got = {}
    for net in tree.iter("net"):
        name = net.get("name").lstrip("/")
        nodes = sorted((n.get("ref"), n.get("pin")) for n in net.iter("node"))
        if not name.startswith("unconnected"):
            got[name] = nodes
    want = {k: sorted(v) for k, v in nets().items()}
    assert got == want
