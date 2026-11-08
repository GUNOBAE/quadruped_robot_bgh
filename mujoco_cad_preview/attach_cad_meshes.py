"""Attach 37 Fusion 360 STL occurrences to the existing BGH MuJoCo linkage preview.

Place this file in the user's existing quadruped_v01 folder, beside
RUN_PREVIEW.cmd and output/quadruped_preview.xml.

Input: Quadruped_CAD_Meshes.zip exported by Fusion 360.
Output: output/cad_preview.xml, output/cad_meshes/*.stl,
        output/cad_fit_report.json.

VISUAL KINEMATIC PREVIEW ONLY. NOT a physics-validated walking model.
This program does not modify the original ZIP, CAD or baseline XML.
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import sys
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
STL_DTYPE = np.dtype([
    ("normal", "<f4", (3,)),
    ("vertex", "<f4", (3, 3)),
    ("attribute", "<u2"),
])


def vec(text: str) -> np.ndarray:
    return np.array([float(v) for v in text.split()], dtype=np.float64)


def read_stl(raw: bytes):
    if len(raw) >= 84:
        n = struct.unpack_from("<I", raw, 80)[0]
        if 84 + n * STL_DTYPE.itemsize == len(raw):
            a = np.frombuffer(raw, dtype=STL_DTYPE, offset=84, count=n)
            if n == 0:
                raise ValueError("STL has no triangles")
            return a["vertex"].astype(np.float64)
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError("Invalid binary/ASCII STL") from exc
    coordinates = re.findall(
        r"^\s*vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)",
        text,
        re.M,
    )
    if not coordinates or len(coordinates) % 3:
        raise ValueError("No valid ASCII STL triangles")
    return np.asarray(coordinates, dtype=np.float64).reshape(-1, 3, 3)


def write_stl(tris: np.ndarray) -> bytes:
    u = tris[:, 1] - tris[:, 0]
    v = tris[:, 2] - tris[:, 0]
    normals = np.cross(u, v)
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-20)
    records = np.zeros(len(tris), dtype=STL_DTYPE)
    records["vertex"] = tris.astype("<f4")
    records["normal"] = normals.astype("<f4")
    return (
        b"BGH CAD Visual Mesh".ljust(80, b" ")
        + struct.pack("<I", len(records))
        + records.tobytes()
    )


def fit_frame(triangles: np.ndarray, bbox: dict, transform: list,
              max_fit_error: float = 0.08):
    """Compare Fusion's potentially conservative AABB correctly.

    Fusion's Occurrence.boundingBox can be the world AABB of the
    8 transformed corners of the component's *local bounding box*.
    The AABB of transformed STL vertices is often smaller for
    diagonally oriented parts, even when the geometry is correct.

    Compare both types; do not resize, rotate, or translate any STL
    to artificially satisfy the check.
    """
    points = triangles.reshape(-1, 3)
    lo = np.asarray(bbox["min"], dtype=np.float64)
    hi = np.asarray(bbox["max"], dtype=np.float64)
    if np.any(lo > hi):
        raise ValueError("Reversed Fusion bounding box")
    mat = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = mat[:3, :3]
    translation = mat[:3, 3]
    denom = max(float(np.linalg.norm(hi - lo)), 2.0)
    candidates = []
    for unit, factor in (("mm", 0.1), ("cm", 1.0), ("m", 100.0), ("inch", 2.54)):
        local = points * factor
        local_min = local.min(axis=0)
        local_max = local.max(axis=0)
        corners = np.array([
            (x, y, z)
            for x in (local_min[0], local_max[0])
            for y in (local_min[1], local_max[1])
            for z in (local_min[2], local_max[2])
        ], dtype=np.float64)
        for frame in ("assembly", "component"):
            if frame == "assembly":
                world_vertices = local
                world_corners = corners
            else:
                world_vertices = local @ rotation.T + translation
                world_corners = corners @ rotation.T + translation

            tight_min = world_vertices.min(axis=0)
            tight_max = world_vertices.max(axis=0)
            conservative_min = world_corners.min(axis=0)
            conservative_max = world_corners.max(axis=0)
            def score(low, high):
                return float(np.linalg.norm(
                    np.concatenate((low - lo, high - hi))
                ) / denom)

            tight_score = score(tight_min, tight_max)
            conservative_score = score(conservative_min, conservative_max)
            selected = ("transformed_local_bbox"
                        if conservative_score < tight_score
                        else "world_vertex_bbox")
            score_value = min(conservative_score, tight_score)
            # The tight geometry must lie inside the Fusion AABB.
            # Allow a tiny tolerance for tessellation / floating precision.
            outside_cm = float(max(
                np.max(lo - tight_min),
                np.max(tight_max - hi),
                0.0
            ))
            candidates.append({
                "error_ratio": score_value,
                "tight_bbox_error_ratio": tight_score,
                "conservative_bbox_error_ratio": conservative_score,
                "bounds_method": selected,
                "stl_units": unit,
                "scale_to_cm": factor,
                "source_frame": frame,
                "outside_fusion_bbox_cm": outside_cm,
                "predicted_bbox_cm": {
                    "min": (conservative_min if selected == "transformed_local_bbox"
                            else tight_min).tolist(),
                    "max": (conservative_max if selected == "transformed_local_bbox"
                            else tight_max).tolist(),
                },
            })
    candidates.sort(key=lambda c: (
        c["error_ratio"], c["outside_fusion_bbox_cm"]
    ))
    chosen = dict(candidates[0])
    if chosen["outside_fusion_bbox_cm"] > 0.03:
        raise ValueError(
            "STL vertices lie outside Fusion bounding box by {:.4g} cm".format(
                chosen["outside_fusion_bbox_cm"]
            )
        )
    chosen["alignment_method"] = chosen["bounds_method"]
    chosen["review_required"] = bool(chosen["error_ratio"] > max_fit_error)
    chosen["center_correction_cm"] = [0.0, 0.0, 0.0]
    chosen["diagnostic_candidates"] = [{
        k: v for k, v in item.items() if k != "predicted_bbox_cm"
    } for item in candidates[:4]]
    return chosen


def read_body_positions(xml_root: ET.Element):
    worldbody = xml_root.find("worldbody")
    if worldbody is None:
        raise ValueError("Base MJCF has no worldbody")
    bodies, positions = {}, {}

    def visit(body, parent):
        name = body.get("name")
        if not name:
            raise ValueError("Unnamed MuJoCo body")
        if any(body.get(k) for k in ("quat", "euler", "axisangle")):
            raise ValueError("Unexpected rotated body: " + name)
        origin = parent + vec(body.get("pos", "0 0 0"))
        bodies[name] = body
        positions[name] = origin
        for child in body.findall("body"):
            visit(child, origin)

    for body in worldbody.findall("body"):
        visit(body, np.zeros(3))
    return bodies, positions


def find_zip(path_arg: str | None):
    choices = (
        [Path(path_arg).expanduser()] if path_arg else
        [HERE / "Quadruped_CAD_Meshes.zip", Path.home() / "Downloads" / "Quadruped_CAD_Meshes.zip"]
    )
    for path in choices:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError(
        "Put Quadruped_CAD_Meshes.zip beside this script, or pass --zip PATH"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zip", help="Explicit Fusion STL ZIP path")
    parser.add_argument("--max-fit-error", type=float, default=0.08)
    args = parser.parse_args()
    if not 0 < args.max_fit_error < 1:
        parser.error("--max-fit-error must be between 0 and 1")
    zip_path = find_zip(args.zip)

    xml_file = HERE / "output" / "quadruped_preview.xml"
    mapping_file = HERE / "output" / "model_manifest.json"
    audit_file = HERE / "audit_excerpt_reference.json"
    for file in (xml_file, mapping_file, audit_file):
        if not file.is_file():
            raise FileNotFoundError(
                "Existing quadruped_v01 preview prerequisite missing: " + str(file)
            )

    xml_tree = ET.parse(xml_file)
    root = xml_tree.getroot()
    bodies, positions = read_body_positions(root)
    mapping = json.loads(mapping_file.read_text(encoding="utf-8"))["body_names"]
    audit = json.loads(audit_file.read_text(encoding="utf-8"))
    rigid = {
        name for group in audit["rigid_groups"]
        for name in group["occurrences"]
    }
    component_to_body = {k: v for k, v in mapping.items() if k != "BASE"}
    for name in rigid:
        component_to_body[name] = mapping["BASE"]
    if len(component_to_body) != 37:
        raise ValueError(
            f"Expected 37 Fusion instances in baseline mapping, got {len(component_to_body)}"
        )

    reports, converted, errors = [], [], []
    with zipfile.ZipFile(zip_path) as archive:
        corrupt = archive.testzip()
        if corrupt:
            raise ValueError("Corrupt ZIP member: " + corrupt)
        names = set(archive.namelist())
        if "mesh_manifest.json" not in names:
            raise ValueError("Missing mesh_manifest.json from Fusion export")
        manifest = json.loads(archive.read("mesh_manifest.json"))
        rows = manifest.get("occurrences", [])
        if len(rows) != 37:
            errors.append(f"Expected 37 occurrences, received {len(rows)}")
        used = set()
        for row in rows:
            name = row.get("occurrence", "")
            file = row.get("file", "")
            entry = {
                "index": row.get("index"),
                "occurrence": name,
                "filename": file,
                "fusion_status": row.get("status"),
            }
            reports.append(entry)
            if name in used:
                errors.append("Repeated occurrence: " + name)
                continue
            used.add(name)
            if name not in component_to_body:
                errors.append("Unknown occurrence: " + name)
                continue
            if row.get("status") != "ok":
                errors.append("Export status not OK for " + name)
                continue
            if file not in names:
                errors.append("Missing STL for " + name + ": " + file)
                continue
            if not row.get("bounds_cm") or not row.get("transform_cm"):
                errors.append("Missing Fusion bbox/transform for " + name)
                continue

            try:
                tris = read_stl(archive.read(file))
                fit = fit_frame(
                    tris, row["bounds_cm"], row["transform_cm"], args.max_fit_error
                )
                entry.update(fit)
                entry["triangles"] = int(len(tris))
                if fit["error_ratio"] > args.max_fit_error:
                    errors.append(
                        f"Mesh bounds mismatch: {name}, error={fit['error_ratio']:.4g}"
                    )
                    continue
                scale = fit["scale_to_cm"]
                if fit["source_frame"] == "assembly":
                    world_cm = tris * scale
                else:
                    mat = np.asarray(row["transform_cm"]).reshape(4, 4)
                    world_cm = tris * scale @ mat[:3, :3].T + mat[:3, 3]
                # Keep CAD geometry exactly as exported (no scaling beyond units,
                # no recentering, and no distortion).
                body_id = component_to_body[name]
                if body_id not in bodies:
                    raise ValueError("Unknown target body: " + body_id)
                local_m = world_cm * 0.01 - positions[body_id]
                converted.append((name, body_id, write_stl(local_m)))
            except Exception as exc:
                entry["error"] = str(exc)
                errors.append(f"{name}: {exc}")

    output = HERE / "output"
    output.mkdir(exist_ok=True)
    report = {
        "source_zip": str(zip_path),
        "source_design": manifest.get("source_design"),
        "parts_reported": len(reports),
        "converted": len(converted),
        "errors": errors,
        "parts": reports,
        "recentered_parts": [p["occurrence"] for p in reports if p.get("alignment_method") == "center_translation_only"],
        "warning": "Visual-only kinematic preview. Not a gravity/contact/RL model. All center-corrected parts require visual comparison against Fusion.",
    }
    report_path = output / "cad_fit_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    if errors or len(converted) != 37:
        print("CAD import halted. See " + str(report_path))
        for error in errors[:40]:
            print(" - " + error)
        raise RuntimeError(f"Only {len(converted)}/37 meshes passed validation")

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("inertiagrouprange", "0 0")
    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        insertion = next(
            (i for i, el in enumerate(list(root)) if el.tag == "worldbody"),
            len(root),
        )
        root.insert(insertion, asset)

    for body in bodies.values():
        for geom in body.findall("geom"):
            geom.set("rgba", "0 0 0 0")
            geom.set("group", "0")
        for site in body.findall("site"):
            site.set("rgba", "0 0 0 0")
            site.set("group", "5")

    meshdir = output / "cad_meshes"
    meshdir.mkdir(exist_ok=True)
    for idx, (name, body_id, raw) in enumerate(converted):
        mesh_name = f"fusion_part_{idx:02d}"
        filename = mesh_name + ".stl"
        (meshdir / filename).write_bytes(raw)
        ET.SubElement(
            asset, "mesh", name=mesh_name, file=filename
        )
        label = name.lower()
        rgba = (
            "0.27 0.29 0.33 1" if body_id == mapping["BASE"] else
            "0.75 0.77 0.79 1" if "femur" in label or "_a" in label else
            "0.58 0.61 0.65 1" if "tibia" in label else
            "0.39 0.42 0.46 1"
        )
        ET.SubElement(
            bodies[body_id], "geom",
            name=mesh_name + "_visual",
            type="mesh", mesh=mesh_name, rgba=rgba,
            contype="0", conaffinity="0", density="0", group="1",
        )

    ET.indent(root, space="  ")
    final_xml = output / "cad_preview.xml"
    xml_tree.write(final_xml, encoding="utf-8", xml_declaration=True)
    print("SUCCESS: 37 Fusion CAD parts attached.")
    if report["recentered_parts"]:
        print("ATTENTION: {} parts were recentered without rotation or stretching; check against Fusion CAD.".format(len(report["recentered_parts"])))
    print("MuJoCo XML:", final_xml)
    print("Fit report:", report_path)
    print("NOTE: This is NOT a physics-validated walking model.")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        import traceback
        print("CAD IMPORT FAILED:", error, file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
