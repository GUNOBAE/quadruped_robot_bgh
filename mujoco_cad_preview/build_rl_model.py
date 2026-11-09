"""Build a SIMULATION-ONLY soft-contact-foot variant of Physics V1.

Silicone is approximated by contact friction + compliant contact constraints,
NOT physically modeled as deformable silicone. Requires later calibration.
No hardware I/O. Original physics_v1.xml and CAD meshes are not modified.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"


def build(source: Path, dest: Path, mu: float = 1.2, tc: float = 0.025,
          damping: float = 1.0):
    if not source.is_file():
        raise FileNotFoundError(f"Physics V1 is missing: {source}")
    if not (0.2 <= mu <= 3.0 and 0.005 <= tc <= .08 and .2 <= damping <= 3):
        raise ValueError("Contact parameters outside guarded ranges")
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "BGH_RL_V0_SIMULATION_ONLY")
    base = root.find(".//freejoint[@name='base_free']")
    actuators = root.findall("./actuator/position")
    toes = [g for g in root.findall(".//geom")
            if g.get("name", "").startswith("foot_collision_")]
    floor = root.find(".//geom[@name='visual_floor']")
    if base is None or len(toes) != 4 or len(actuators) != 12 or floor is None:
        raise RuntimeError("Expected Physics V1 free base + 4 toes + 12 actuators + floor")
    if len(root.findall("./equality/connect")) != 16:
        raise RuntimeError("Expected 16 equality connect constraints for 8 loops")
    contact_attrs = {
        "friction": f"{mu:g} .015 .001",
        "solref": f"{tc:g} {damping:g}",
        "solimp": ".75 .95 .001 .5 2",
        "condim": "3",
        "contype": "1",
        "conaffinity": "1",
    }
    for toe in toes:
        toe.attrib.update(contact_attrs)
    floor.set("friction", f"{mu:g} .015 .001")
    floor.set("solref", f"{tc:g} {damping:g}")
    floor.set("solimp", ".75 .95 .001 .5 2")
    option = root.find("option")
    if option is None or option.get("gravity") != "0 0 -9.81":
        raise RuntimeError("Expected Physics V1 with gravity -9.81")
    if option.get("timestep") not in (".001", "0.001"):
        raise RuntimeError("Expected 1 ms physics timestep")
    ET.indent(root, "  ")
    dest.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dest, encoding="utf-8", xml_declaration=True)
    report = {
        "source": str(source),
        "output": str(dest),
        "simulation_only": True,
        "real_silicone_properties_measured": False,
        "warning": "A soft contact proxy is NOT a viscoelastic silicone finite-element model.",
        "silicone_contact_proxy": {"sliding_friction": mu,
                                    "solver_time_constant_s": tc,
                                    "damping_ratio": damping},
        "toes": [g.get("name") for g in toes],
        "actuator_mapping_verified_against_real_hardware": False,
    }
    (dest.parent / "rl_v0_contact_config.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print("RL contact model created:", dest)
    print("Silicone proxy =", report["silicone_contact_proxy"])
    print("SIMULATION ONLY: no measured material model or verified servo channels.")
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, default=OUT / "physics_v1.xml")
    p.add_argument("--output", type=Path, default=OUT / "physics_rl_v0.xml")
    p.add_argument("--friction", type=float, default=1.2)
    p.add_argument("--contact-time", type=float, default=.025)
    p.add_argument("--damping", type=float, default=1.)
    a = p.parse_args()
    build(a.source, a.output, a.friction, a.contact_time, a.damping)


if __name__ == "__main__":
    main()
