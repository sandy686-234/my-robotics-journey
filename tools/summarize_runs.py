#!/usr/bin/env python3
import json, csv, pathlib
from json import JSONDecodeError

root = pathlib.Path("runs")
rows = []
skipped = []

if not root.exists():
    print("No runs/ directory found.")
    exit(0)

for d in sorted(p for p in root.glob("*") if p.is_dir()):
    cert_path = d / "pdc_certificate.json"
    trace_path = d / "trace_constrained.csv"

    feasible = ""
    min_slack = ""
    miss = 0
    phi_all = ""

    if cert_path.exists():
        try:
            cert = json.loads(cert_path.read_text())
            feasible = cert.get("feasible", "")
            windows = cert.get("windows", [])
            if windows:
                try:
                    min_slack = min(w.get("margin", float("inf")) for w in windows)
                    if min_slack == float("inf"):
                        min_slack = ""
                except Exception:
                    min_slack = ""
        except (JSONDecodeError, OSError) as e:
            skipped.append(f"{d.name}: bad certificate ({e})")
    else:
        skipped.append(f"{d.name}: no pdc_certificate.json (legacy run)")

    if trace_path.exists():
        try:
            with trace_path.open() as f:
                rdr = csv.DictReader(f)
                last = None
                for r in rdr:
                    last = r
                    if r.get("event") == "miss":
                        miss += 1
                if last:
                    phi_all = last.get("phi_all_bool", "")
        except Exception as e:
            skipped.append(f"{d.name}: bad trace ({e})")

    rows.append({
        "run": d.name,
        "feasible": feasible,
        "min_slack_s": round(float(min_slack), 3) if isinstance(min_slack, (int, float)) and min_slack != "" else "",
        "miss": miss,
        "phi_all_bool_last": phi_all,
        "has_cert": int(cert_path.exists()),
        "has_trace": int(trace_path.exists()),
    })

out = pathlib.Path("runs_summary.csv")
with out.open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["run"])
    w.writeheader()
    w.writerows(rows)

print(f"Wrote {out} with {len(rows)} rows")
if skipped:
    print("Skipped notes:")
    for s in skipped:
        print(" -", s)
