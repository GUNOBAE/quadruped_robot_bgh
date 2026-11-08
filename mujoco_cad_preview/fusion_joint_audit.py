"""Fusion 360 Script: export full joint/occurrence audit for the robot dog.

Run INSIDE Fusion 360 -> Utilities -> Scripts and Add-Ins.
This reads the open design. It never changes or saves the CAD assembly.
Output: ~/Downloads/quadruped_joint_audit.json.
"""
import adsk.core
import adsk.fusion
import json
import os
import traceback


def safe_property(obj, name):
    try:
        return getattr(obj, name)
    except Exception:
        return None


def collection_items(coll):
    if coll is None:
        return []
    try:
        return [coll.item(i) for i in range(coll.count)]
    except Exception:
        try:
            return list(coll)
        except Exception:
            return []


def point3(p):
    try:
        return [p.x, p.y, p.z]
    except Exception:
        return None


def matrix4(t):
    try:
        return list(t.asArray())
    except Exception:
        return None


def occ_name(occ):
    return safe_property(occ, "fullPathName") or "<root>"


def joint_record(j, type_name):
    m = safe_property(j, "jointMotion")
    limit = safe_property(m, "rotationLimits")
    return {
        "name": safe_property(j, "name"),
        "kind": type_name,
        "motion_type": safe_property(m, "objectType"),
        "joint_type_code": str(safe_property(m, "jointType")),
        "occurrence_1": occ_name(safe_property(j, "occurrenceOne")),
        "occurrence_2": occ_name(safe_property(j, "occurrenceTwo")),
        "is_suppressed": safe_property(j, "isSuppressed"),
        "rotation_axis_vector": point3(safe_property(m, "rotationAxisVector")),
        "rotation_axis_mode": str(safe_property(m, "rotationAxis")),
        "rotation_limits": {
            "minimum_enabled": safe_property(limit, "isMinimumValueEnabled"),
            "maximum_enabled": safe_property(limit, "isMaximumValueEnabled"),
            "minimum_rad": safe_property(limit, "minimumValue"),
            "maximum_rad": safe_property(limit, "maximumValue"),
        } if limit else None,
        "as_built_transform_cm": matrix4(safe_property(j, "transform")),
        "geometry_1_transform_cm": matrix4(safe_property(j, "geometryOneTransform")),
        "geometry_2_transform_cm": matrix4(safe_property(j, "geometryTwoTransform")),
    }


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("Open ASSAMBLE_A.f3d in Design workspace first.")
            return
        root = design.rootComponent
        occs = []
        for occ in collection_items(root.allOccurrences):
            physical = safe_property(occ, "physicalProperties")
            bounds = safe_property(occ, "boundingBox")
            occs.append({
                "path": occ_name(occ),
                "component": occ.component.name,
                "is_grounded": safe_property(occ, "isGrounded"),
                "is_ground_to_parent": safe_property(occ, "isGroundToParent"),
                "transform_cm": matrix4(safe_property(occ, "transform2")),
                "mass_kg_cad_estimate": safe_property(physical, "mass"),
                "center_of_mass_cm": point3(safe_property(physical, "centerOfMass")),
                "bbox_min_cm": point3(safe_property(bounds, "minPoint")),
                "bbox_max_cm": point3(safe_property(bounds, "maxPoint")),
            })
        joints = [joint_record(j, "Joint") for j in collection_items(root.allJoints)]
        built = [joint_record(j, "AsBuiltJoint") for j in collection_items(root.allAsBuiltJoints)]
        rigid = [{
            "name": safe_property(g, "name"),
            "occurrences": [
                occ_name(o) for o in collection_items(safe_property(g, "occurrences"))
            ],
        } for g in collection_items(root.allRigidGroups)]
        report = {
            "model_name": root.name,
            "unit_notes": "Fusion internal length = cm, mass = kg, angle = rad",
            "summary": {
                "occurrences": len(occs), "joints": len(joints),
                "as_built_joints": len(built), "rigid_groups": len(rigid),
            },
            "occurrences": occs, "joints": joints,
            "as_built_joints": built, "rigid_groups": rigid,
        }
        output = os.path.join(os.path.expanduser("~"), "Downloads", "quadruped_joint_audit.json")
        with open(output, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        ui.messageBox(
            "Joint audit completed!\n" + str(report["summary"]) +
            "\n" + output
        )
    except Exception:
        if ui:
            ui.messageBox(traceback.format_exc())
        else:
            print(traceback.format_exc())
