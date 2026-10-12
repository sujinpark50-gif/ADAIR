# -*- coding: utf-8 -*-
"""ugv/tools/run_provenance.py — (WSL 안에서) 한 시험 실행의 실제 모델·월드 해석 증거를 JSON 으로 남긴다 (차량 개조 독립 검증 P2 지적 반영).

    python3 -m ugv.tools.run_provenance --run-id <TAG> --spawn <스폰 파일> --out <json> [--state ~/.adair-ugv]

- $STATE_DIR/procs.tsv 에서 역할이 "<역할>:<RUN_ID>" 인 프로세스만 고른다 (다른 실행의 PX4 를 집지 않음). PID 의 시작 시각이 기록과 같은지도 확인.
- PX4 프로세스마다 환경의 GZ_SIM_RESOURCE_PATH·PX4_GZ_MODEL_NAME·PX4_SYS_AUTOSTART 를 읽고, 스폰 파일의 모델(adair_ugv / adair_firetruck / x500)을
  그 경로 순서대로(Gazebo 의 model:// 해석 순서) 찾아 처음 일치한 model.sdf 의 경로·내용 해시(줄바꿈 정규화 sha1)를 기록한다.
- gz 서버 명령줄의 월드 파일 경로·내용 해시, 그 월드가 include 하는 지형 타일(model://T..)을 gz 서버의 모델 경로에서 찾아
  heightmap.png·model.sdf 해시와 전체 요약 해시(tiles_digest)도 함께.
- Gazebo 에 실제 생성된 모델의 관절 목록 (`gz model -m <이름> -j`): 관절 수·prismatic(서스펜션) 수 — 파일이 아니라 실행 중 모델 기준.
- 찾지 못한 항목은 null 과 이유를 남긴다 (빈 값 해시를 쓰지 않는다).
- PX4 없이 띄운 실행(NO_PX4=1, 정지·대각 시험)은 PX4 기록이 없으므로 gz 서버 프로세스 환경의 GZ_SIM_RESOURCE_PATH 로 모델을 찾는다
  (resolved_via = gz_server_env). 이때 차량 이름은 <모델>_<인스턴스>. PX4 기록이 있으면 예전과 같다.
한계: 모델 파일 해석은 PX4 프로세스 환경의 경로 순서를 따라간 것이고 Gazebo 내부 해석을 직접 읽은 것은 아니다 — 관절 조회로 구조를 교차 확인.
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path


def sha1_text(p):
    try:
        return hashlib.sha1(Path(p).read_bytes().replace(b"\r", b"")).hexdigest()
    except OSError:
        return None


def starttime(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().split()[21]
    except OSError:
        return None


def environ(pid):
    try:
        return dict(kv.split("=", 1) for kv in Path(f"/proc/{pid}/environ").read_bytes().decode(errors="replace").split("\0") if "=" in kv)
    except OSError:
        return {}


def gz_joints(model_name, partition):
    """실행 중 Gazebo 의 모델 관절 (이름·종류). 실패하면 None."""
    gz_env = os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh")
    try:
        r = subprocess.run(["bash", "-c", f". '{gz_env}' 2>/dev/null; timeout 25 gz model -m '{model_name}' -j"], capture_output=True, text=True,
                           timeout=40, env={**os.environ, "GZ_PARTITION": partition, "GZ_IP": "127.0.0.1"})
    except (OSError, subprocess.TimeoutExpired):
        return None
    names = re.findall(r"- Name: (\S+)", r.stdout)
    types = re.findall(r"- Type: (\S+)", r.stdout)
    if not names:
        return None
    return [{"name": n, "type": t} for n, t in zip(names, types)]


def tiles_of(world_file, resource_path):
    try:
        txt = Path(world_file).read_text(encoding="utf-8")
    except OSError:
        return None
    out = []
    for t in sorted(set(re.findall(r"model://(T\d+_\d+)", txt))):
        d = next((Path(x) / t for x in resource_path if (Path(x) / t / "heightmap.png").exists()), None)
        out.append({"tile": t, "dir": str(d) if d else None, "heightmap_sha1": sha1_text(d / "heightmap.png") if d else None,
                    "model_sdf_sha1": sha1_text(d / "model.sdf") if d else None})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--spawn", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--state", default=os.path.expanduser(os.getenv("STATE_DIR", "~/.adair-ugv")))
    a = ap.parse_args()
    procs = []
    tsv = Path(a.state) / "procs.tsv"
    if tsv.exists():
        for line in tsv.read_text().splitlines():
            f = line.split("\t")
            if len(f) >= 4 and f[0].endswith(":" + a.run_id):
                procs.append({"role": f[0].rsplit(":", 1)[0], "pid": int(f[1]), "start": f[2], "partition": f[3],
                              "alive_same": starttime(f[1]) == f[2], "cmd": f[4].strip() if len(f) > 4 else ""})
    models = []
    for line in Path(a.spawn).read_text().replace("\r", "").splitlines():
        p = line.split()
        if len(p) >= 4:
            models.append({"instance": p[0], "model": p[1], "resource_id": p[3]})
    out = {"run_id": a.run_id, "procs_tsv": str(tsv), "procs": procs, "vehicles": [], "world": None}
    gz = next((x for x in procs if x["role"] == "gz"), None)
    if gz:
        wf = next((t for t in gz["cmd"].split() if t.endswith(".sdf")), None)
        out["world"] = {"file": wf, "content_sha1": sha1_text(wf) if wf else None}
        gz_rp = [d for d in environ(gz["pid"]).get("GZ_SIM_RESOURCE_PATH", "").split(":") if d]
        tl = tiles_of(wf, gz_rp) if wf else None
        out["world"]["tiles"] = tl
        out["world"]["tiles_digest"] = (hashlib.sha1("".join(f"{x['tile']}{x['heightmap_sha1']}{x['model_sdf_sha1']}" for x in tl).encode()).hexdigest()
                                        if tl and all(x["heightmap_sha1"] for x in tl) else None)
    for m in models:
        px = next((x for x in procs if x["role"] == f"px4-{m['instance']}"), None)
        rec = {**m, "px4_pid": px["pid"] if px else None}
        if not px and gz and gz["alive_same"] and m["model"] != "x500":
            env = environ(gz["pid"])
            rp = [d for d in env.get("GZ_SIM_RESOURCE_PATH", "").split(":") if d]
            rec.update({"resolved_via": "gz_server_env (PX4 없음, NO_PX4)", "resource_path": rp})
            hit = next((d for d in rp if (Path(d) / m["model"] / "model.sdf").exists()), None)
            if hit:
                f = Path(hit) / m["model"] / "model.sdf"
                rec.update({"model_file": str(f), "model_content_sha1": sha1_text(f)})
            else:
                rec["error"] = "gz 서버 모델 경로에서 model.sdf 를 찾지 못함"
            js = gz_joints(f"{m['model']}_{m['instance']}", gz["partition"])
            rec["gz_joints"] = js
            rec["gz_joint_count"] = None if js is None else len(js)
            rec["gz_prismatic_count"] = None if js is None else sum(j["type"] == "prismatic" for j in js)
            if js is None:
                rec["error"] = "Gazebo 모델 관절 조회 실패"
        elif not px:
            rec["error"] = "이 실행 ID 의 PX4 프로세스 기록 없음"
        elif not px["alive_same"]:
            rec["error"] = "PX4 프로세스가 없거나 시작 시각 불일치 (PID 재사용)"
        else:
            env = environ(px["pid"])
            rp = [d for d in env.get("GZ_SIM_RESOURCE_PATH", "").split(":") if d]
            rec.update({"px4_gz_model_name": env.get("PX4_GZ_MODEL_NAME"), "px4_autostart": env.get("PX4_SYS_AUTOSTART"),
                        "resource_path": rp})
            hit = next((d for d in rp if (Path(d) / m["model"] / "model.sdf").exists()), None)
            if hit:
                f = Path(hit) / m["model"] / "model.sdf"
                rec.update({"model_file": str(f), "model_content_sha1": sha1_text(f)})
            js = gz_joints(env.get("PX4_GZ_MODEL_NAME") or f"{m['model']}_{m['instance']}", px["partition"])
            rec["gz_joints"] = js
            rec["gz_joint_count"] = None if js is None else len(js)
            rec["gz_prismatic_count"] = None if js is None else sum(j["type"] == "prismatic" for j in js)
            if js is None and m["model"] != "x500":
                rec["error"] = "Gazebo 모델 관절 조회 실패"
            if m["model"] == "x500" and not hit:
                rec["model_file"] = "PX4 기본 x500 (저장소 밖)"
            elif not hit:
                rec["error"] = "모델 경로에서 model.sdf 를 찾지 못함"
        out["vehicles"].append(rec)
    Path(a.out).write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    bad = [v for v in out["vehicles"] if v.get("error")]
    ok = not bad and out["world"] and out["world"].get("tiles_digest")
    print("PROVENANCE_OK" if ok else f"PROVENANCE_INCOMPLETE {[v.get('error') for v in bad]} world={bool(out['world'])} tiles={bool(out['world'] and out['world'].get('tiles_digest'))}")


if __name__ == "__main__":
    main()
