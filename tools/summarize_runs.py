#!/usr/bin/env python3
import json,csv,pathlib
rows=[]; root=pathlib.Path("runs")
for d in sorted(p for p in root.glob("*") if p.is_dir()):
    cert=json.loads((d/"pdc_certificate.json").read_text())
    feasible=cert.get("feasible",False)
    windows=cert.get("windows",[])
    min_slack=min((w["margin"] for w in windows), default=None)
    miss=0; phi_all=""
    tr=d/"trace_constrained.csv"
    if tr.exists():
        import csv as _csv
        with tr.open() as f:
            r=_csv.DictReader(f)
            for row in r:
                if row["event"]=="miss": miss+=1
                phi_all=row.get("phi_all_bool",phi_all)
    rows.append({"run":d.name,"feasible":feasible,
                "min_slack_s":round(min_slack,3) if min_slack is not None else "",
                "miss":miss,"phi_all_bool_last":phi_all})
out=pathlib.Path("runs_summary.csv")
with out.open("w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys() if rows else ["run"])
    w.writeheader(); w.writerows(rows)
print("Wrote", out, "rows=",len(rows))
