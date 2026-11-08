"""Fusion 360 Script: export all 37 robot CAD occurrences into a ZIP.

Run from Fusion 360 -> Utilities -> Scripts and Add-Ins, not plain VS Code.
Read-only: does not change the original .f3d model.
Output: ~/Downloads/Quadruped_CAD_Meshes.zip.
"""
import adsk.core
import adsk.fusion
import json
import os
import re
import traceback
import zipfile


def safe_name(text):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", text).strip("_")[:52] or "part"


def p3(point):
    return [point.x, point.y, point.z] if point else None


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox("Open ASSAMBLE_A.f3d in the Design workspace first.")
            return

        download = os.path.join(os.path.expanduser("~"), "Downloads")
        os.makedirs(download, exist_ok=True)
        folder = os.path.join(download, "Quadruped_CAD_Meshes")
        os.makedirs(folder, exist_ok=True)

        exporter = design.exportManager
        root = design.rootComponent
        results = []
        for i in range(root.allOccurrences.count):
            occ = root.allOccurrences.item(i)
            name = occ.fullPathName
            filename = "{:02d}_{}.stl".format(i, safe_name(name))
            target = os.path.join(folder, filename)
            record = {
                "index": i,
                "occurrence": name,
                "component": occ.component.name,
                "file": filename,
                "status": "pending",
            }
            try:
                bounds = occ.boundingBox
                record["bounds_cm"] = {
                    "min": p3(bounds.minPoint),
                    "max": p3(bounds.maxPoint),
                }
                record["transform_cm"] = list(occ.transform2.asArray())

                options = exporter.createSTLExportOptions(occ, target)
                options.sendToPrintUtility = False
                options.isBinaryFormat = True
                options.isOneFilePerBody = False
                success = exporter.execute(options)

                if (success and os.path.isfile(target)
                        and os.path.getsize(target) > 84):
                    record["status"] = "ok"
                else:
                    record["status"] = "not_created"
            except Exception as err:
                record["status"] = "error"
                record["error"] = str(err)
            results.append(record)

        manifest = {"source_design": root.name, "occurrences": results}
        manifest_file = os.path.join(folder, "mesh_manifest.json")
        with open(manifest_file, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2, ensure_ascii=False)
        zip_file = os.path.join(download, "Quadruped_CAD_Meshes.zip")
        with zipfile.ZipFile(
            zip_file, "w", compression=zipfile.ZIP_DEFLATED
        ) as archive:
            archive.write(manifest_file, arcname="mesh_manifest.json")
            for row in results:
                if row["status"] == "ok":
                    archive.write(
                        os.path.join(folder, row["file"]), arcname=row["file"]
                    )

        passed = sum(r["status"] == "ok" for r in results)
        ui.messageBox(
            "Quadruped CAD export complete!\n"
            + str(passed) + "/" + str(len(results))
            + " STL files exported.\n\n"
            + zip_file
        )
    except Exception:
        if ui is not None:
            ui.messageBox(traceback.format_exc())
        else:
            print(traceback.format_exc())
