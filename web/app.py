# -*- coding: utf-8 -*-
"""웹 관제판: 강원 지형 + 산불 CA + 드론 실시간 + 화재 출동."""
import os, uuid
import httpx
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
import config
config.FIRE_CONNECTION_MODE = "real"
from connectors import fire_connector
import gz_bridge as gb
import math
from ugv.road_graph import RoadGraph
from ugv.graph_data_demo import NODES, ROADS
from connectors import ugv_connector
_RG = RoadGraph(NODES, ROADS)
_NODE_LL = {n["node_id"]:(n["lat"],n["lon"]) for n in NODES}
def _nearest_node(lat, lon):
    best=None; bd=1e18
    for nid,(la,lo) in _NODE_LL.items():
        d=(la-lat)**2+(lo-lon)**2
        if d<bd: bd=d; best=nid
    return best

UAV_AGENT = os.environ.get("UAV_AGENT_URL", "http://localhost:8000")
UAV_ID    = os.environ.get("UAV_ID", "A-uav1")
WIND_MS   = float(os.environ.get("WEB_WIND_MS", "3.0"))
PREVIEW   = os.path.join(os.path.dirname(__file__), "..", "gazebo", "kangwon_terrain_preview.png")
COLS, ROWS = 308, 236

from fastapi.staticfiles import StaticFiles
app = FastAPI(title="ADAIR 강원 관제판")
app.mount("/static", StaticFiles(directory=os.path.join(os.path.dirname(__file__),"static")), name="static")
_tick = 0

class FlyReq(BaseModel):
    col: int; row: int

@app.get("/api/preview")
def preview():
    if os.path.exists(PREVIEW): return FileResponse(PREVIEW, media_type="image/png")
    raise HTTPException(404, "no preview")

@app.get("/api/env")
def env():
    """환경 CA 한 틱 진행 → 불타는 셀·위험셀 반환(격자 x,y)."""
    global _tick
    es = fire_connector.get_environment_state(float(_tick)); _tick += 1
    fire = [{"x": c["x"], "y": c["y"]} for c in es.fire_cells]
    risk = [{"x": c["x"], "y": c["y"]} for c in es.risk_zone[:150]]
    return {"tick": _tick, "fire": fire, "risk": risk,
            "wind_speed": es.wind_speed, "spread": es.spread_direction}

@app.get("/api/state")
async def state():
    async with httpx.AsyncClient(timeout=10) as cx:
        r = await cx.get(f"{UAV_AGENT}/uav/{UAV_ID}/state"); r.raise_for_status()
        return r.json()

@app.get("/api/cell")
def cell(lat: float, lon: float):
    try:
        x, y = gb.latlon_to_epsg(lat, lon)
        return {"col": int((x-gb.GRID_LEFT)/gb.GRID_RES), "row": int((gb.GRID_TOP-y)/gb.GRID_RES)}
    except Exception as e:
        return {"error": str(e)}

@app.get("/api/ugv")
def ugv(col: int = 60, row: int = 70):
    # 자원 목록(mock): UGV/소방차 위치·capability
    try:
        res = ugv_connector.get_ugv_status("A")
    except Exception:
        res = []
    units=[]
    for r in res:
        units.append({"id": r.resource_id, "type": r.resource_type,
                      "lat": r.location_lat, "lon": r.location_lon,
                      "cap": r.capability or {}})
    # 화재셀 → 위경도 → 최근접 도로노드(goal)
    flat, flon = gb.grid_cell_to_latlon(col, row)
    goal = _nearest_node(flat, flon)
    routes=[]
    for u in units:
        start = _nearest_node(u["lat"], u["lon"])
        try:
            rr = _RG.find_route(start, goal)
            path = [{"lat": _NODE_LL[nid][0], "lon": _NODE_LL[nid][1]} for nid in getattr(rr,"path",[]) or []]
            routes.append({"id": u["id"], "reachable": rr.reachable,
                           "eta_sec": getattr(rr,"eta_s",None), "path": path})
        except Exception as e:
            routes.append({"id": u["id"], "reachable": False, "eta_sec": None, "path": [], "err": str(e)})
    # 차단 도로 수
    blocked = sum(1 for rd in ROADS if rd.get("blocked"))
    return {"fire_cell":{"col":col,"row":row,"lat":round(flat,6),"lon":round(flon,6)},
            "goal_node": goal, "units": units, "routes": routes, "blocked_roads": blocked}

@app.post("/api/extinguish")
def extinguish(cell: dict):
    """자원이 도착한 화재셀을 진화 처리(관측으로 UNBURNED 반영)."""
    try:
        col=int(cell["col"]); row=int(cell["row"])
        lat,lon=gb.grid_cell_to_latlon(col,row)
        x,y=gb.latlon_to_epsg(lat,lon)
        api=fire_connector._engine()   # 강원 CA 엔진
        r=api.apply_observation({"reporter_id":"RESPONSE","world_x":x,"world_y":y,"fire_state":"UNBURNED"})
        return {"ok": bool(r.get("success")), "cell": [col,row]}
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ===== 자동 폐루프 =====
_AUTO = {"on": False, "target": None, "assigned": False, "t": 0}

@app.post("/api/auto/toggle")
def auto_toggle():
    _AUTO["on"] = not _AUTO["on"]
    if _AUTO["on"]:
        _AUTO["target"] = None; _AUTO["assigned"] = False
    return {"on": _AUTO["on"]}

@app.post("/api/auto/tick")
async def auto_tick():
    if not _AUTO["on"]:
        return {"on": False, "events": []}
    ev = []
    if _AUTO["target"] is None:
        es = fire_connector.get_environment_state(float(_AUTO["t"])); _AUTO["t"] += 1
        if not es.fire_cells:
            return {"on": True, "events": ["화재 없음 - 대기"]}
        best = max(es.fire_cells, key=lambda c: c.get("risk_score", 0.0))
        _AUTO["target"] = {"col": best["x"], "row": best["y"]}; _AUTO["assigned"] = False
        ev.append("감지: 셀(%d,%d)" % (best["x"], best["y"]))
    tgt = _AUTO["target"]
    if not _AUTO["assigned"]:
        try:
            lat, lon = gb.grid_cell_to_latlon(tgt["col"], tgt["row"])
            target = {"lat": round(lat,6), "lon": round(lon,6), "alt_m_amsl": round(gb.terrain_elev(tgt["col"],tgt["row"]),1)}
            base = {"task_id":"AUTO-"+uuid.uuid4().hex[:6], "decision_id":"AD-"+uuid.uuid4().hex[:6], "target": target, "observation_type":"THERMAL"}
            async with httpx.AsyncClient(timeout=20) as cx:
                r = (await cx.post(f"{UAV_AGENT}/uav/{UAV_ID}/evaluate", json={**base,"wind_ms":WIND_MS})).json()
                if r.get("verdict") == "ACCEPT":
                    await cx.post(f"{UAV_AGENT}/uav/{UAV_ID}/execute", json=base)
                    _AUTO["assigned"] = True; ev.append("드론 출동 ACCEPT")
                else:
                    ev.append("드론 " + str(r.get("verdict")))
        except Exception as e:
            ev.append("출동 오류: " + str(e))
    return {"on": True, "events": ev, "target": tgt}

@app.post("/api/auto/done")
def auto_done():
    t = _AUTO.get("target")
    if t:
        try:
            lat,lon = gb.grid_cell_to_latlon(t["col"], t["row"]); x,y = gb.latlon_to_epsg(lat,lon)
            fire_connector._engine().apply_observation({"reporter_id":"AUTO","world_x":x,"world_y":y,"fire_state":"UNBURNED"})
        except Exception:
            pass
    _AUTO["target"] = None; _AUTO["assigned"] = False
    return {"ok": True}

@app.post("/api/fly")
async def fly(req: FlyReq):
    lat, lon = gb.grid_cell_to_latlon(req.col, req.row)
    target = {"lat": round(lat,6), "lon": round(lon,6), "alt_m_amsl": round(gb.terrain_elev(req.col,req.row),1)}
    tid = "WEB-"+uuid.uuid4().hex[:8]; did = "DEC-"+uuid.uuid4().hex[:8]
    base = {"task_id": tid, "decision_id": did, "target": target, "observation_type": "THERMAL"}
    async with httpx.AsyncClient(timeout=30) as cx:
        ev = (await cx.post(f"{UAV_AGENT}/uav/{UAV_ID}/evaluate", json={**base,"wind_ms":WIND_MS})).json()
        if ev.get("verdict") != "ACCEPT":
            return {"ok": False, "verdict": ev.get("verdict"), "reason": ev.get("reason"), "target": target}
        ex = (await cx.post(f"{UAV_AGENT}/uav/{UAV_ID}/execute", json=base)).json()
    return {"ok": True, "task_id": ex.get("task_id", tid), "target": target, "eta_sec": ev.get("eta_sec")}

@app.get("/", response_class=HTMLResponse)
def index(): return HTML

HTML = """<!DOCTYPE html><html lang=ko><head><meta charset=utf-8><title>ADAIR 강원 관제판</title>
<style>
body{margin:0;font-family:system-ui,'Malgun Gothic';background:#0f1720;color:#e6f2ef}
header{padding:12px 18px;background:#14273f;font-weight:800}
.wrap{display:flex;gap:14px;padding:14px;flex-wrap:wrap}
.map{position:relative;flex:1 1 560px;min-width:360px}
.map img{width:100%;display:block;border-radius:10px;border:1px solid #2a3a4d}
#ov{position:absolute;left:0;top:0;width:100%;height:100%}
.panel{flex:1 1 280px;min-width:260px;background:#16202b;border:1px solid #2a3a4d;border-radius:10px;padding:14px}
h3{margin:.2em 0 .5em;color:#8fd7c9}.row{display:flex;gap:8px;margin:8px 0;align-items:center}
input{width:74px;padding:7px;border-radius:8px;border:1px solid #3a4a5d;background:#0f1720;color:#e6f2ef}
button{padding:9px 14px;border:0;border-radius:8px;font-weight:700;cursor:pointer;color:#fff}
#fly{background:#e0483a}#clr{background:#3a5c99}
.kv{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #22303f}
.kv b{color:#8fd7c9}.log{font-family:monospace;font-size:12px;color:#9fe;white-space:pre-wrap;background:#0d1f14;border:1px solid #0C8E7E;border-radius:8px;padding:8px;margin-top:8px}
small{color:#6f8a96}
</style></head><body>
<header>🔥🚁 ADAIR — 강원 지형 · 산불 · 드론 실시간</header>
<div class=wrap>
 <div class=map>
   <img id=terrain src=/api/preview>
   <canvas id=ov></canvas>
   <small>빨강=연소중 · 주황=위험 · 청록=드론 · 파랑선=비행궤적 · 노랑=출동 타깃</small>
 </div>
 <div class=panel>
   <h3>출동</h3>
   <div class=row>col <input id=col type=number value=60> row <input id=row type=number value=70>
     <button id=fly>🔥 출동</button><button id=clr>궤적 지우기</button><button id=auto style="background:#0C8E7E">자동 시작</button></div>
   <h3>산불(CA)</h3>
   <div class=kv><b>연소 셀</b><span id=nfire>-</span></div>
   <div class=kv><b>바람</b><span id=wind>-</span></div>
   <h3>드론</h3>
   <div class=kv><b>lat</b><span id=lat>-</span></div>
   <div class=kv><b>lon</b><span id=lon>-</span></div>
   <div class=kv><b>고도</b><span id=alt>-</span></div>
   <div class=kv><b>모드</b><span id=mode>-</span></div>
   <div class=kv><b>타깃거리</b><span id=dist>-</span></div>
   <h3>지상자원(UGV)</h3><div class=kv><b>상태</b><span id=ugvinfo>-</span></div>
   <h3>자원 목록</h3><div id=fleet style="font-size:12px"></div>
   <div class=log id=log>대기…</div>
 </div>
</div>
<script>
const COLS=308,ROWS=236,$=id=>document.getElementById(id);
const img=$('terrain'),ov=$('ov');
let fire=[],risk=[],drone=null,tgt=null,trail=[];
function fit(){ov.width=img.clientWidth;ov.height=img.clientHeight;}
img.onload=fit;window.onresize=fit;
function P(c,r){return [(c+0.5)/COLS*ov.width,(r+0.5)/ROWS*ov.height];}
let _cellCache={};
function llCell(lat,lon){const k=lat.toFixed(5)+','+lon.toFixed(5);
  if(_cellCache[k])return _cellCache[k];
  // 서버 /api/cell 은 async라, 근사: DEM transform 역산을 클라에서(간이)
  const GRID_LEFT=298096,GRID_TOP=612773,RES=90; // EPSG:5186 근사 — 데모용
  // 위경도→EPSG 근사는 서버가 정확. 여기선 캐시 미스 시 중앙 반환 후 서버 보정
  return _cellCache[k]||{col:154,row:118};}
// 로버 주행 애니: 유닛별 경로 진행률(0~1) 상태
let _driveT={}, _extd={};          // {unitId: 진행률}
let _driveStart={};      // {unitId: 시작 시각}
function pathPointAt(rt,u){
  // rt.path(노드 col/row) 를 따라 u(0~1) 위치 보간
  const pts=(rt.path||[]).filter(p=>p.col!=null);
  if(pts.length<2) return pts[0]||null;
  const seg=(pts.length-1)*Math.max(0,Math.min(1,u));
  const i=Math.min(pts.length-2,Math.floor(seg)); const f=seg-i;
  return {col:pts[i].col+(pts[i+1].col-pts[i].col)*f,
          row:pts[i].row+(pts[i+1].row-pts[i].row)*f};
}
function hav(a,b,c,d){const R=6371000,r=Math.PI/180;
 const dp=(c-a)*r,dl=(d-b)*r,A=Math.sin(dp/2)**2+Math.cos(a*r)*Math.cos(c*r)*Math.sin(dl/2)**2;
 return 2*R*Math.asin(Math.sqrt(A));}
function draw(){const g=ov.getContext('2d');g.clearRect(0,0,ov.width,ov.height);
 risk.forEach(c=>{const[x,y]=P(c.x,c.y);g.fillStyle='rgba(224,140,40,.5)';g.fillRect(x-2,y-2,4,4);});
 fire.forEach(c=>{const[x,y]=P(c.x,c.y);g.fillStyle='rgba(224,60,40,.9)';g.beginPath();g.arc(x,y,3.5,0,7);g.fill();});
 if(tgt){const[x,y]=P(tgt.col,tgt.row);g.strokeStyle='#ffd23a';g.lineWidth=3;g.beginPath();g.arc(x,y,9,0,7);g.stroke();
   g.fillStyle='#ffd23a';g.beginPath();g.arc(x,y,3,0,7);g.fill();}
 if(trail.length>1){g.strokeStyle='#4aa3ff';g.lineWidth=2.5;g.beginPath();
   trail.forEach((p,i)=>{const[x,y]=P(p.col,p.row);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();}
// UGV 자원·도로경로
 if(window.__ugv){const U=window.__ugv;
   (U.routes||[]).forEach(rt=>{ if(rt.path&&rt.path.length>1){
     g.strokeStyle=rt.reachable?'rgba(230,150,40,.9)':'rgba(150,150,150,.6)';g.lineWidth=3;g.beginPath();
     rt.path.forEach((p,i)=>{if(p.col==null)return;const[x,y]=P(p.col,p.row);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();}});
   U._pos={};
   (U.units||[]).forEach(u=>{
     const rt=(U.routes||[]).find(r=>r.id===u.id);
     let pos=null;
     if(rt&&rt.reachable&&rt.path&&rt.path.length>1){pos=pathPointAt(rt,_driveT[u.id]||0);}
     if(!pos&&u.col!=null)pos={col:u.col,row:u.row};
     if(!pos)return; U._pos[u.id]=pos;
     const[x,y]=P(pos.col,pos.row);
     const fire=(u.type==='FIRE_ENGINE');
     const sz=fire?11:8;
     g.fillStyle=fire?'#ff3b30':'#e8912a';g.fillRect(x-sz,y-sz,sz*2,sz*2);
     g.strokeStyle='#fff';g.lineWidth=2;g.strokeRect(x-sz,y-sz,sz*2,sz*2);
     if(fire){g.fillStyle='#fff';g.font='bold 11px sans-serif';g.textAlign='center';g.fillText('🚒',x,y+4);}
     g.fillStyle='#cfe';g.font='11px sans-serif';g.textAlign='left';g.fillText(u.id,x+sz+3,y+4);
     if((_driveT[u.id]||0)>=0.999){g.strokeStyle='#ffd23a';g.lineWidth=2.5;g.beginPath();g.arc(x,y,sz+5,0,7);g.stroke();}
   });
 }
 if(drone){const[x,y]=P(drone.col,drone.row);
   if(tgt){const[tx,ty]=P(tgt.col,tgt.row);g.strokeStyle='rgba(74,163,255,.5)';g.setLineDash([6,6]);g.lineWidth=2;
     g.beginPath();g.moveTo(x,y);g.lineTo(tx,ty);g.stroke();g.setLineDash([]);}
   g.fillStyle='#00d1b2';g.beginPath();g.arc(x,y,7,0,7);g.fill();
   g.strokeStyle='rgba(0,209,178,.5)';g.lineWidth=4;g.beginPath();g.arc(x,y,12,0,7);g.stroke();}}
async function tickEnv(){try{const e=await(await fetch('/api/env')).json();fire=e.fire;risk=e.risk;
 $('nfire').textContent=fire.length;$('wind').textContent=e.wind_speed+' m/s '+e.spread;draw();}catch(e){}}
let lastLL=null;
async function tickState(){try{const s=await(await fetch('/api/state')).json();const p=s.position||{};
 $('lat').textContent=p.lat;$('lon').textContent=p.lon;$('alt').textContent=(p.alt_m_amsl)+' m';
 $('mode').textContent=s.flight_mode;
 if(p.lat!=null){const g=await(await fetch('/api/cell?lat='+p.lat+'&lon='+p.lon)).json();
   if(g.col!=null){drone=g;
     // 궤적: 위치가 유의미하게 바뀌면 점 추가
     const last=trail[trail.length-1];
     if(!last||Math.abs(last.col-g.col)+Math.abs(last.row-g.row)>=1){trail.push({col:g.col,row:g.row});if(trail.length>400)trail.shift();}
     draw();}
   if(tgt&&tgt.lat!=null){$('dist').textContent=Math.round(hav(p.lat,p.lon,tgt.lat,tgt.lon))+' m';}
 }}catch(e){}}
async function tickUgv(){try{const col=+$('col').value,row=+$('row').value;
  const u=await(await fetch('/api/ugv?col='+col+'&row='+row)).json();
  // 유닛 위경도→격자 정확 변환(서버 /api/cell)
  for(const un of (u.units||[])){const g=await(await fetch('/api/cell?lat='+un.lat+'&lon='+un.lon)).json();if(g.col!=null){un.col=g.col;un.row=g.row;}}
  for(const rt of (u.routes||[])){for(const p of rt.path){const g=await(await fetch('/api/cell?lat='+p.lat+'&lon='+p.lon)).json();if(g.col!=null){p.col=g.col;p.row=g.row;}}}
  // 경로가 바뀐 유닛은 주행 리셋
  (u.routes||[]).forEach(rt=>{const prev=(window.__ugv&&(window.__ugv.routes||[]).find(x=>x.id===rt.id));
    const changed=!prev||JSON.stringify(prev.path)!==JSON.stringify(rt.path);
    if(changed){_driveStart[rt.id]=null;_driveT[rt.id]=0;}});
  window.__ugv=u; draw();
  // 자원 목록 패널
  const rows=(u.units||[]).map(un=>{
    const rt=(u.routes||[]).find(r=>r.id===un.id)||{};
    const t=_driveT[un.id]||0;
    const icon=un.type==='FIRE_ENGINE'?'🚒':(un.type==='UGV'?'🚙':'🛩');
    const st=!rt.reachable?'<span style=color:#e88>도달불가</span>':(t>=0.999?'<span style=color:#8f8>도착</span>':('주행 '+Math.round(t*100)+'%'));
    const eta=rt.eta_sec!=null?(' · ETA '+Math.round(rt.eta_sec)+'s'):'';
    return '<div style="padding:3px 0;border-bottom:1px solid #22303f">'+icon+' '+un.id+' — '+st+eta+'</div>';
  }).join('');
  document.getElementById('fleet').innerHTML=rows||'<small>자원 없음</small>';
  // 도착한 자원의 목표 화재셀 진화 요청(중복 방지)
  (u.routes||[]).forEach(rt=>{ if(rt.reachable && (_driveT[rt.id]||0)>=0.999 && !_extd[rt.id]){
     _extd[rt.id]=true;
     fetch('/api/extinguish',{method:'POST',headers:{'Content-Type':'application/json'},
       body:JSON.stringify({col:+$('col').value,row:+$('row').value})}).catch(()=>{});
     $('log').textContent=rt.id+' 화재 도착 → 진화 반영';if(window.__autoActive&&window.__autoActive()){window.__autoDone();_extd={};}
  }});
  const reach=(u.routes||[]).filter(r=>r.reachable).length, tot=(u.routes||[]).length;
  const eta=(u.routes||[]).map(r=>r.eta_sec).filter(x=>x!=null);
  $('ugvinfo').textContent='자원 '+tot+'대 · 도달 '+reach+'/'+tot+(eta.length?(' · ETA '+Math.round(Math.min(...eta))+'s'):'')+' · 차단도로 '+u.blocked_roads;
}catch(e){}}
function driveStep(){
  const U=window.__ugv; if(U){ (U.routes||[]).forEach(rt=>{
    if(rt.reachable&&rt.path&&rt.path.length>1){
      if(_driveStart[rt.id]==null)_driveStart[rt.id]=performance.now();
      const dur=Math.max(6,Math.min(60,(rt.eta_sec||30)))*1000; // 화면용 6~60s로 클램프
      _driveT[rt.id]=Math.min(1,(performance.now()-_driveStart[rt.id])/dur);
    }
  }); draw(); }
  requestAnimationFrame(driveStep);
}
requestAnimationFrame(driveStep);
setInterval(tickEnv,1500);setInterval(tickState,1000);setInterval(tickUgv,3000);setTimeout(()=>{fit();tickEnv();tickState();tickUgv();},600);
$('fly').onclick=async()=>{const col=+$('col').value,row=+$('row').value;
 $('log').textContent='출동('+col+','+row+')…';
 const r=await(await fetch('/api/fly',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({col,row})})).json();
 if(r.ok){tgt={col,row,lat:r.target.lat,lon:r.target.lon};trail=[];_driveT={};_driveStart={};_extd={};
   $('log').textContent='✅ ACCEPT\\n타깃 '+JSON.stringify(r.target)+'\\ntask '+r.task_id;}
 else{$('log').textContent='⛔ '+(r.verdict||'')+' '+(r.reason||'');}
 draw();};
$('clr').onclick=()=>{trail=[];draw();};
</script><script src="/static/auto.js"></script></body></html>"""