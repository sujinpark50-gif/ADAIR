# -*- coding: utf-8 -*-
"""환경 Connector — mock + real(강원 CA 직접 연동). run_real_px4 용."""
import os, sys, contextlib
import config
from interfaces.schema import EnvironmentState

_ENV_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "environment"))
_api=None; _tick=0

@contextlib.contextmanager
def _in_env():
    prev=os.getcwd(); ins=False
    try:
        if _ENV_DIR not in sys.path: sys.path.insert(0,_ENV_DIR); ins=True
        os.chdir(_ENV_DIR); yield
    finally:
        os.chdir(prev)
        if ins:
            try: sys.path.remove(_ENV_DIR)
            except ValueError: pass

def _engine():
    global _api
    if _api is not None: return _api
    cfg=getattr(config,"FIRE_ENV_CONFIG_PATH",os.path.join(_ENV_DIR,"config","environment_config.yaml"))
    seed=getattr(config,"FIRE_SIM_SEED",42); ign=getattr(config,"FIRE_IGNITION_XY",(19,142))
    with _in_env():
        from src.environment.grid import EnvironmentGrid
        from src.environment.fire_model import WildfireCAEngine
        from src.environment.api import EnvironmentModelAPI
        rel=os.path.relpath(cfg,_ENV_DIR)
        grid=EnvironmentGrid(config_path=rel,seed=seed)
        eng=WildfireCAEngine(grid=grid,config_path=rel,seed=seed)
        api=EnvironmentModelAPI(grid=grid,ca_engine=eng)
        ix,iy=int(ign[0]),int(ign[1])
        if not(0<=ix<grid.cols and 0<=iy<grid.rows): ix,iy=grid.cols//2,grid.rows//2
        eng.ignite_at(ix,iy)
    _api=api; return _api

def get_environment_state(timestamp: float) -> EnvironmentState:
    if config.FIRE_CONNECTION_MODE=="mock": return _mock_state(timestamp)
    global _tick
    api=_engine()
    if _tick>0: api.step()
    _tick+=1
    fire=[]
    for loc in api.get_fire_locations():
        x,y=loc["x"],loc["y"]; fire.append({"cell_id":f"{x}_{y}","x":x,"y":y,
            "fire_state":"BURNING","risk_score":round(float(loc.get("risk_score",0.0)),3)})
    thr=getattr(config,"FIRE_RISK_ZONE_THRESHOLD",0.4); mx=getattr(config,"FIRE_RISK_ZONE_MAX",300)
    cand=[c for c in api.grid.cells if c.fire_state.value=="UNBURNED" and c.risk_score>=thr]
    cand.sort(key=lambda c:c.risk_score,reverse=True)
    rz=[{"cell_id":f"{c.x}_{c.y}","x":c.x,"y":c.y,"risk_score":round(float(c.risk_score),3)} for c in cand[:mx]]
    # 위경도 보강(gz_bridge 있으면)
    try:
        import gz_bridge as gb
        for c in fire+rz:
            lat,lon=gb.grid_cell_to_latlon(c["x"],c["y"]); c["location_lat"]=round(lat,6); c["location_lon"]=round(lon,6)
    except Exception: pass
    sp=api.get_spread_direction()
    return EnvironmentState(timestamp=timestamp,fire_cells=fire,risk_zone=rz,
        wind_speed=float(sp.get("wind_speed_ms",0.0)),wind_direction=float(sp.get("wind_direction_deg",0.0)),
        spread_direction=sp.get("primary_cardinal","N"),env_updated=False)

def update_environment(env_state: EnvironmentState, observations: list) -> EnvironmentState:
    if config.FIRE_CONNECTION_MODE=="mock":
        if observations: env_state.env_updated=True
        return env_state
    api=_engine(); applied=False
    for obs in (observations or []):
        lat=getattr(obs,"location_lat",None); lon=getattr(obs,"location_lon",None)
        if lat is None or lon is None: continue
        try:
            import gz_bridge as gb
            x,y=gb.latlon_to_epsg(float(lat),float(lon))
            r=api.apply_observation({"reporter_id":getattr(obs,"resource_id","UAV"),
                "world_x":x,"world_y":y,"fire_state":"BURNING"})
            if r.get("success"): applied=True
        except Exception as e: print("[fire] 관측 반영 스킵:",e)
    env_state.env_updated=applied
    return env_state

def _mock_state(timestamp):
    import random
    return EnvironmentState(timestamp=timestamp,
        fire_cells=[{"cell_id":"50_50","x":50,"y":50,"fire_state":"BURNING","risk_score":0.8}],
        risk_zone=[{"cell_id":"51_50","x":51,"y":50,"risk_score":0.6}],
        wind_speed=3.0,wind_direction=270.0,spread_direction="E",env_updated=False)
