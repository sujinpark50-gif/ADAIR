# -*- coding: utf-8 -*-
"""총괄 서버 내장 우선순위 판단 화면 (/board).

기존 웹관제(web/app.py, 통합 담당)는 수정하지 않는다. 통합팀은 같은 데이터를
/priority/board, /priority/mode, /priority/choose 로 가져가 기존 화면에 붙이면 된다.

화면 디자인: 발표용 소방 상황실 형태 (2026-10-06, 통합 담당 이원규). 데이터·버튼·API 호출과
표시 문구(모의 관측·시험 주입값 등 출처 표시 포함)는 이전 화면과 같고 배치와 모양만 바꿨다.
"""

BOARD_HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ADAIR 산불 대응 상황판</title>
<style>
:root{--bg:#0b0f14;--panel:#121821;--panel2:#18202b;--line:#253041;--ink:#e8edf3;--mute:#8b97a8;
 --fire:#ff4d2e;--fire2:#ff8a3d;--amber:#ffb020;--green:#2fd27c;--blue:#4aa3ff;--cyan:#22d3ee;--red:#ff3b3b}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--ink);
 font-family:system-ui,-apple-system,"Malgun Gothic","Apple SD Gothic Neo",sans-serif}
.num{font-variant-numeric:tabular-nums}
/* 상단 */
.top{display:grid;grid-template-columns:1fr auto 1fr;align-items:center;gap:16px;padding:12px 20px;
 background:linear-gradient(90deg,#1a0d0a,#121821 40%,#121821);border-bottom:2px solid var(--fire)}
.brand{display:flex;align-items:center;gap:12px}
.logo{width:42px;height:42px;border-radius:10px;display:grid;place-items:center;font-size:24px;
 background:radial-gradient(circle at 50% 60%,#ff6a2e,#9b1c0c);box-shadow:0 0 18px rgba(255,77,46,.45)}
.brand h1{margin:0;font-size:20px;letter-spacing:.5px}.brand small{display:block;color:var(--mute);font-size:12px;margin-top:2px}
.clock{text-align:center}.clock .t{font-size:34px;font-weight:800;line-height:1}.clock .d{color:var(--mute);font-size:12px;margin-top:4px}
.ctrl{display:flex;justify-content:flex-end;flex-wrap:wrap;gap:8px;align-items:center}
.seg{display:flex;border:1px solid var(--line);border-radius:10px;overflow:hidden}
.seg button{border:0;background:var(--panel2);color:var(--mute);padding:9px 12px;font-weight:600;cursor:pointer;font-size:13px}
.seg button.on{background:var(--blue);color:#fff}
.btn{border:1px solid var(--line);background:var(--panel2);color:var(--ink);padding:9px 12px;border-radius:10px;cursor:pointer;font-weight:600;font-size:13px}
.btn:hover{border-color:var(--blue)}
.live{display:flex;align-items:center;gap:6px;font-size:12px;color:var(--mute)}
.dot{width:9px;height:9px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green)}.dot.off{background:var(--red);box-shadow:0 0 8px var(--red)}
/* 경보 띠 */
.alert{display:flex;align-items:center;gap:14px;padding:12px 20px;font-weight:800;font-size:18px;border-bottom:1px solid var(--line)}
.alert .ic{font-size:22px}.alert small{font-weight:500;font-size:13px;opacity:.85}
.alert.lv3{background:linear-gradient(90deg,rgba(255,59,59,.35),rgba(255,59,59,.08));color:#ffd9d9;animation:blink 1.6s infinite}
.alert.lv2{background:linear-gradient(90deg,rgba(255,138,61,.30),rgba(255,138,61,.06));color:#ffe2cc}
.alert.lv1{background:linear-gradient(90deg,rgba(255,176,32,.22),rgba(255,176,32,.05));color:#ffefc7}
.alert.lv0{background:linear-gradient(90deg,rgba(47,210,124,.18),rgba(47,210,124,.04));color:#d2f7e3}
@keyframes blink{50%{background:linear-gradient(90deg,rgba(255,59,59,.18),rgba(255,59,59,.04))}}
/* KPI */
.kpis{display:grid;grid-template-columns:repeat(6,1fr);gap:12px;padding:14px 20px}
.kpi{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px 14px;position:relative;overflow:hidden}
.kpi:before{content:"";position:absolute;left:0;top:0;bottom:0;width:4px;background:var(--c,var(--mute))}
.kpi .l{color:var(--mute);font-size:12px}.kpi .v{font-size:30px;font-weight:800;margin-top:4px}.kpi .s{color:var(--mute);font-size:11px}
/* 본문 */
.main{display:grid;grid-template-columns:minmax(0,2fr) minmax(320px,1fr);gap:14px;padding:0 20px 14px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:14px;margin-bottom:14px;overflow:hidden}
.ph{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:11px 14px;background:var(--panel2);border-bottom:1px solid var(--line)}
.ph h2{margin:0;font-size:15px;display:flex;align-items:center;gap:8px}.ph .sub{color:var(--mute);font-size:11px;font-weight:400}
.pb{padding:12px 14px}
.empty{color:var(--mute);font-size:13px;padding:6px 0}
.panel.decide{border-color:var(--amber);box-shadow:0 0 0 1px rgba(255,176,32,.3),0 0 24px rgba(255,176,32,.12)}
.panel.decide .ph{background:rgba(255,176,32,.12)}
/* 출동 우선순위 */
.q{display:grid;grid-template-columns:54px minmax(0,1.5fr) 110px 120px minmax(0,1.2fr) minmax(0,1.1fr) minmax(0,1fr) 92px;
 gap:10px;align-items:center;padding:10px 12px;border-bottom:1px solid var(--line);border-left:4px solid var(--qc,var(--line))}
.q.head{color:var(--mute);font-size:11px;padding-top:8px;padding-bottom:8px;border-left-color:transparent;background:rgba(255,255,255,.02)}
.rank{font-size:26px;font-weight:900;color:var(--mute);text-align:center}.q.top1 .rank{color:var(--fire2)}
.task{font-weight:700;font-size:15px}.task .ic{margin-right:6px}.tid{color:var(--mute);font-size:10px;font-weight:400;display:block;margin-top:2px}
.pill{display:inline-block;padding:3px 9px;border-radius:999px;font-size:12px;font-weight:700;border:1px solid currentColor;white-space:nowrap}
.p-run{color:var(--fire2);animation:pulse 1.4s infinite}.p-ok{color:var(--green)}.p-hold{color:var(--amber)}.p-bad{color:var(--red)}.p-wait{color:var(--mute)}.p-go{color:var(--blue)}
@keyframes pulse{50%{box-shadow:0 0 0 4px rgba(255,138,61,.18)}}
.hold{display:block;color:var(--amber);font-size:11px;margin-top:3px}
.bar{height:6px;background:#0c1118;border-radius:4px;overflow:hidden;margin-top:4px}.bar i{display:block;height:100%;background:linear-gradient(90deg,var(--amber),var(--fire))}
.human{color:#fff;background:var(--red);padding:2px 8px;border-radius:6px;font-weight:800;font-size:12px}
.mute{color:var(--mute)}.small{font-size:11px}
.obs{font-weight:700}.obs.fire{color:var(--fire)}.obs.clear{color:var(--green)}.obs.warn{color:var(--amber)}
.go{padding:7px 10px;border-radius:8px;border:0;background:var(--fire);color:#fff;font-weight:800;cursor:pointer;font-size:12px;width:100%}
/* 카드 */
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:10px}
.card{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:12px;border-top:3px solid var(--cc,var(--line))}
.card.first{outline:2px solid var(--blue)}
.ct{font-weight:800;margin-bottom:8px;display:flex;justify-content:space-between;gap:8px;align-items:baseline}
.row{display:flex;justify-content:space-between;gap:12px;font-size:13px;padding:3px 0;border-bottom:1px dashed rgba(255,255,255,.05)}
.row>span:first-child{color:var(--mute);white-space:nowrap}.row>span:last-child,.row>b:last-child{text-align:right}
.extra{margin-top:8px;padding-top:6px;border-top:1px solid var(--line)}
.ai{margin:0 0 10px;padding:9px 12px;border-radius:10px;background:rgba(74,163,255,.12);border:1px solid rgba(74,163,255,.35);font-size:13px}
.ai.off{background:transparent;border-color:var(--line);color:var(--mute)}
.ghead{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px;font-weight:700}
.tag{font-size:12px;padding:2px 9px;border-radius:999px;border:1px solid currentColor}
.tag.wait{color:var(--amber)}.tag.auto{color:var(--blue)}.tag.ok{color:var(--green)}.tag.k{color:var(--mute)}
.card button.go{margin-top:10px;padding:9px}
/* 화재·기상 */
.frow{display:flex;align-items:center;gap:10px;padding:8px 0;border-bottom:1px solid var(--line)}
.frow .cell{font-weight:900;font-size:16px;min-width:64px}.frow .meta{color:var(--mute);font-size:11px}
.fi{width:12px;height:12px;border-radius:50%;flex:none}
.fi.fire{background:var(--fire);box-shadow:0 0 10px var(--fire);animation:pulse2 1.2s infinite}.fi.warn{background:var(--amber)}.fi.clear{background:var(--green)}
@keyframes pulse2{50%{transform:scale(1.35)}}
.wx{display:grid;grid-template-columns:44px 1fr;gap:10px;align-items:center;padding:8px 0;border-bottom:1px solid var(--line)}
.compass{width:44px;height:44px;border-radius:50%;border:1px solid var(--line);display:grid;place-items:center;position:relative;background:#0c1118}
.compass:after{content:"N";position:absolute;top:1px;font-size:8px;color:var(--mute)}
.compass svg{transition:transform .6s}
.wx .nm{font-weight:800}.wx .v{font-size:13px}.wx .meta{color:var(--mute);font-size:11px}
.sec{color:var(--mute);font-size:11px;font-weight:700;margin:12px 0 4px;letter-spacing:.5px}
/* 하단 */
.foot{padding:0 20px 18px;color:var(--mute);font-size:12px;line-height:1.6}
.foot .box{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:10px 14px}
.foot b{color:var(--ink)}
.warnbox{margin-top:8px;padding:8px 12px;border:1px solid var(--amber);border-radius:10px;color:#ffe2a8;background:rgba(255,176,32,.08)}
@media (max-width:1100px){.main{grid-template-columns:1fr}.kpis{grid-template-columns:repeat(3,1fr)}
 .q{grid-template-columns:40px 1fr 1fr}.q>*:nth-child(n+4){grid-column:2/-1}.q.head{display:none}}
</style></head><body>
<div class="top">
 <div class="brand"><div class="logo">🔥</div><div><h1>ADAIR 산불 대응 상황판</h1><small id="sub">총괄 우선순위 판단</small></div></div>
 <div class="clock"><div class="t num" id="clk">--:--</div><div class="d" id="clkd">시나리오 시각</div></div>
 <div class="ctrl"><div class="seg" id="mode"></div>
  <button class="btn" onclick="post('/dispatch_pending')">대기 임무 배정 실행</button><button class="btn" onclick="post('/poll')">진행 갱신</button>
  <span class="live"><span class="dot" id="live"></span><span id="livet">연결</span></span></div>
</div>
<div class="alert lv0" id="alert"><span class="ic">✅</span><span>상황 확인 중…</span></div>
<div class="kpis" id="kpis"></div>
<div class="main">
 <div>
  <div class="panel" id="decideP"><div class="ph"><h2>🧭 지휘 결정 <span class="sub">선택 묶음 · 비슷하거나 엇갈리는 후보</span></h2></div><div class="pb" id="groups"></div></div>
  <div class="panel"><div class="ph"><h2>🚨 출동 우선순위 <span class="sub">전체 판단 순서</span></h2><span class="sub" id="ruleShort"></span></div>
   <div class="q head"><span>순서</span><span>임무</span><span>인명피해</span><span>위험도</span><span>가까운 보호대상</span><span>상태</span><span>배정 자원 · 정찰 결과</span><span>수동</span></div>
   <div id="order"></div></div>
  <div class="panel"><div class="ph"><h2>📡 현장 보고 <span class="sub">정찰·측정 결과</span></h2></div><div class="pb"><div class="cards" id="results"></div></div></div>
 </div>
 <div>
  <div class="panel"><div class="ph"><h2>🔥 화재 현황 <span class="sub">알고 있는 불 · 신고·정찰·관측으로 알게 된 것만</span></h2></div><div class="pb" id="fires"></div></div>
  <div class="panel"><div class="ph"><h2>🌬️ 기상 <span class="sub">관측소 최신 관측 (그 시각까지) · 현장 측정</span></h2></div><div class="pb" id="wx"></div></div>
  <div class="panel"><div class="ph"><h2>🏘️ 확산 예측 · 위협 시설 <span class="sub" id="fcsub">현재 시험 주입값 · 관측으로 계산한 예측 아님</span></h2></div><div class="pb" id="forecast"></div></div>
 </div>
</div>
<div class="foot"><div class="box">
 <div>자동 배정은 생성된 임무의 우선순위를 판단하고 자원을 배정하는 모드입니다. 시뮬레이션 시작은 통합관제에서 제어합니다.</div>
 <div id="rule"></div><div id="prov"></div><div id="gate"></div>
 <div>총괄이 아는 상황 = 신고·정찰·관측으로 알게 된 것만 · 시뮬레이션 진짜 상태 아님</div>
</div></div>
<script>
const MODE_LABEL={AUTO_HIGHER_RISK:"자동 배정 (AI 추천으로 출발)",HUMAN_CHOICE:"수동 선택 (사람이 선택)"};
const KIND_LABEL={SIMILAR:"위험도 비슷",CONFLICT:"기준 엇갈림"};
const STATUS_LABEL={AWAITING_CHOICE:["선택 대기","wait"],PARTIAL_AWAITING_CHOICE:["일부 선택 대기","wait"],AUTO_DECIDED:["자동 출발 (규칙: 위험도 높은 곳 먼저)","auto"],LLM_DECIDED:["자동 출발 (AI 추천, 검증 통과)","auto"],ALL_DISPATCHED:["자원 충분 · 모두 출동","ok"]};
const PURPOSE={PENDING:"대기",HOLD:"보류",EVALUATING:"판단 중",APPROVED:"출동 승인",IN_EXECUTION:"출동 중",COMPLETED:"완료",FAILED:"실패",CANCELLED:"취소"};
const PCLS={PENDING:"p-wait",HOLD:"p-hold",EVALUATING:"p-go",APPROVED:"p-go",IN_EXECUTION:"p-run",COMPLETED:"p-ok",FAILED:"p-bad",CANCELLED:"p-wait"};
const QCOL={PENDING:"#8b97a8",HOLD:"#ffb020",EVALUATING:"#4aa3ff",APPROVED:"#4aa3ff",IN_EXECUTION:"#ff8a3d",COMPLETED:"#2fd27c",FAILED:"#ff3b3b",CANCELLED:"#8b97a8"};
const HOLD={AWAITING_PRIORITY_CHOICE:"사람 선택 대기",NO_FEASIBLE_CANDIDATE:"보낼 자원 없음",HUMAN_PRIORITY_CHOSEN:"사람이 선택함",
 EXECUTION_RESPONSE_LOST:"출동 응답 끊김",PROVIDER_LOST_TASK:"기체가 임무 기록 잃음",PROVIDER_OFFLINE:"기체 연락 두절",
 OBSERVATION_REQUIREMENT_NOT_MET:"관측 범위 부족 · 재관측 필요",ENV_APPLY_NOT_ACKED:"환경 반영 미확인",ARRIVED_OBSERVATION_PENDING:"도착 · 관측 대기",
 TARGET_LOCATION_UNKNOWN:"지도에 위치 없음 · 출동 보류",AREA_COVERAGE_INCOMPLETE:"구역 일부만 관측 · 남은 칸 대기",AREA_EVIDENCE_UNSUPPORTED:"구역 전체 관측 불가 (드론 구역 관측 계약 대기)",
 ENV_APPLY_TIMEOUT:"환경 반영 응답 없음",ENV_APPLY_PENDING:"관측 완료 · 환경 반영 확인 대기 (다시 보내지 않음)",ENV_ACK_UNSUPPORTED_FOR_SENSOR:"환경 반영 확인을 지원하지 않는 관측",RESPONSE_VALIDATION_FAILED:"기체 응답 검증 실패",
 RUN_GATE:"실행(run) 전환 대기",RUN_ENDED_BEFORE_OBSERVATION:"실행(run)이 끝나 관측 없음",OPEN_ATTEMPT_EXISTS:"진행 중인 출동 있음",MANUAL_IDLE_CONFIRMED:"기체 정지 확인됨"};
const TASK_KIND={ENV_SENSE:"환경 측정",RECON:"정찰",MONITOR:"감시",RECHECK:"재확인",GROUND_RECON:"지상 확인",GROUND_SUPPORT:"지상 지원"};
const KIND_IC={ENV_SENSE:"🌡️",RECON:"🔍",MONITOR:"👁️",RECHECK:"🔁",GROUND_RECON:"🚙",GROUND_SUPPORT:"🚒"};
const SITE_TYPE={VILLAGE:"마을",FACILITY:"시설",HOUSE:"주택",SCHOOL:"학교",HOSPITAL:"병원",REST_AREA:"휴게소"};
const RES_TYPE={uav:"드론",ugv:"무인차량",fire:"소방차"};const RES_IC={uav:"🛩️",ugv:"🚙",fire:"🚒"};
const OBS={MEASURED:["측정 완료","clear"],DETECTED:["불 발견","fire"],NOT_DETECTED:["불 없음","clear"],NO_COVERAGE:["관측 범위 밖","warn"],FAILED:["관측 실패","warn"],ROAD_STATUS:["도로 상황 확인","clear"],NONE:["관측값 없음","warn"]};
const STATE_KO={BURNING:"불타는 중",BURNED:"탄 곳"};
const COMPLETION_KO={SIMULATION_TEST:"시뮬레이션 시험 완료 (모의 센서·가짜 환경 확인)",INTEGRATED:"실제 연동 완료"};
const LLM_REASON={API_KEY_MISSING:"API 키 없음",TIMEOUT_NOT_CONFIGURED:"대기시간 미설정",CALL_LIMIT_NOT_CONFIGURED:"호출 한도 미설정",CALL_LIMIT_REACHED:"호출 한도 도달",NOT_CONNECTED:"연결 안 됨"};
const WX_LABEL={wind_ms:["풍속","m/s"],wind_dir_deg:["풍향(불어오는 쪽)","°"],temperature_c:["기온","℃"],humidity_pct:["습도","%"]};
const BASIS_KO={FORECAST_REACHES_RESIDENTIAL:"주거지에 도달 예상",FORECAST_REACHES_OCCUPIED_SITE:"사람 있는 보호대상에 도달 예상"};
const FIRE_KO={REPORTED:["신고됨 (확인 전)","warn"],CONFIRMED:["불 확인","fire"],CONFIRMED_BURNED:["탄 곳 확인","warn"],OBSERVED_CLEAR:["관측 결과 불 없음","clear"],NOT_DETECTED_PARTIAL:["본 범위에 불 없음 (칸 일부만 관측)","warn"]};
const WX_STATUS={EXPIRED:"만료",POLICY_NOT_SET:"유효시간 미설정",OBSERVED_TIME_UNKNOWN:"관측시각 불명",VALUE_INVALID:"값 이상"};
const ENV_KO={FIXTURE:"시험용 가짜 환경 (팀 공유 환경 아님)",TEAM_API:"환경팀 공유 환경 서버",TEAM_ENV_IN_PROCESS:"환경팀 계약 환경 (총괄 안에서 실행 — 관제판과 시계 공유 안 됨)"};
const ANALYSIS_KO={TEST_INJECTED:"시험 주입값 — 관측으로 계산한 예측 아님 (환경팀 분석 기능 대기)",ENV_BELIEF_ANALYSIS:"환경팀 분석 (총괄이 아는 정보로 계산)",ENV_ANALYSIS_UNAVAILABLE:"환경팀 분석 응답 없음"};
const SRC_KO={"119_CALL":"119 신고",EXTERNAL_REPORT:"외부 신고",SIMULATED:"모의 관측",KMA_ASOS_HOURLY:"기상청 관측소(시간자료)",FIXTURE_STATION:"시험 관측소"};
const esc=s=>String(s??"").replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const small=s=>s?` <span class="tid">${esc(s)}</span>`:"";
const $=id=>document.getElementById(id);
const mins=s=>s==null?"-":(s<60?`${Math.round(s)}초`:`${Math.round(s/60)}분`);
const risk=v=>v==null?"점수 없음":v.toFixed(2);
function resKey(id){const m=/^([A-Z]+)-([a-z]+)(\d+)$/.exec(id||"");return m?m[2]:null;}
function resName(id){if(!id)return "-";const m=/^([A-Z]+)-([a-z]+)(\d+)$/.exec(id);return m?`${RES_IC[m[2]]||""} ${m[1]}거점 ${RES_TYPE[m[2]]||m[2]} ${m[3]}호`:esc(id);}
const title=t=>`<span class="ic">${KIND_IC[t.kind]||"📌"}</span>${esc(TASK_KIND[t.kind]||"임무")} · ${esc(t.cell_id||"-")}`;
const human=(v,basis)=>v===true?`<span class="human">예상됨${basis&&basis.length===1&&basis[0]==="FORECAST"?" (확산 예측)":""}</span>`:(v===false?`<span class="mute">없음</span>`:`<span class="mute">정보 없음</span>`);
const near=t=>t.nearest_protected?`${esc(SITE_TYPE[t.nearest_protected.site_type]||"보호대상")} ${Math.round(t.distance_m)}m${small(t.nearest_protected.site_id)}`:`<span class="mute">-</span>`;
function holdText(t){if(!t.hold_reason)return "";const h=t.hold_reason.split(":")[0];return HOLD[h]||(h.startsWith("HANDOVER")?"인계 필요":"사유 기록됨");}
function statusPill(t){if(!t.purpose_status)return "-";const s=PURPOSE[t.purpose_status]||t.purpose_status;const h=holdText(t);
 return `<span class="pill ${PCLS[t.purpose_status]||"p-wait"}">${esc(s)}</span>${h?`<span class="hold">${esc(h)}</span>`:""}`;}
function obsShort(o){if(!o)return '<span class="mute small">정찰 결과 아직 없음</span>';const r=OBS[o.result]||[o.result,"warn"];
 const where=o.detections&&o.detections.length?` (${o.detections.map(d=>esc(d.cell_id)).join(", ")})`:"";return `<span class="obs ${r[1]}">${r[0]}${where}</span>`;}
function completionRow(o){const c=o.completion;return c?`<div class="row"><span>완료 근거</span><span>${COMPLETION_KO[c.evidence_level]||esc(c.evidence_level)}</span></div>`:"";}
function obsDetail(o){return obsDetail0(o)+(o.extras||[]).map(x=>`<div class="extra"><div class="row"><span><b>같은 방문에서 함께 측정</b></span><span></span></div>${obsDetail0(x)}</div>`).join("")+completionRow(o);}
function obsDetail0(o){if(o.sensor_type==="WEATHER"){const v=o.values||{};
  return `<div class="row"><span>결과</span>${obsShort(o)}</div>${Object.keys(WX_LABEL).filter(k=>v[k]!=null).map(k=>`<div class="row"><span>${WX_LABEL[k][0]}</span><b class="num">${v[k]}${WX_LABEL[k][1]}</b></div>`).join("")}
  <div class="row"><span>측정 범위</span><span>${o.scope==="CELL"?"그 칸의 값":"지도 전체 값 (칸별 기상 없음)"}</span></div>
  <div class="row"><span>출처</span><span>모의 측정 (시험용 센서)</span></div>
  <div class="row"><span>환경 반영</span><span>${o.env_apply&&o.env_apply.result==="ACK"?`확인됨 (환경 버전 ${o.env_apply.state_version})`:"확인 안 됨"}</span></div>
  <div class="row"><span>측정 자원</span><span>${resName(o.resource_id)}</span></div>`;}
 const fp=o.footprint;const src=o.source==="SIMULATED"?"모의 관측 (시험용 센서)":(o.source==="UGV_PROVIDER"||!o.is_fire_observation?"무인차량 도로 관측 (화재 관측 아님)":esc(o.source));
 const ack=o.env_apply?(o.env_apply.result==="ACK"?`확인됨 (환경 버전 ${o.env_apply.state_version})`:(o.env_apply.result==="SKIPPED"?"필요 없음":"확인 안 됨")):"없음";
 return `<div class="row"><span>결과</span>${obsShort(o)}</div>
 ${o.detections&&o.detections.length?`<div class="row"><span>발견한 칸</span><span>${o.detections.map(d=>`${esc(d.cell_id)} ${STATE_KO[d.fire_state]||esc(d.fire_state)}`).join(", ")}</span></div>`:""}
 <div class="row"><span>전체를 본 칸</span><span>${o.covered_cells&&o.covered_cells.length?o.covered_cells.map(esc).join(", "):"없음"}</span></div>
 ${o.partial_cells&&o.partial_cells.length?`<div class="row"><span>일부만 본 칸</span><span>${o.partial_cells.map(c=>{const f=(o.cell_coverage||[]).find(x=>x.cell_id===c);return esc(c)+(f?` (${Math.round(f.fraction*100)}%)`:"");}).join(", ")}</span></div>`:""}
 ${o.received_after_purpose?`<div class="row"><span>비고</span><span>임무 ${PURPOSE[o.received_after_purpose]||esc(o.received_after_purpose)} 뒤에 들어온 관측 (완료 근거 아님)</span></div>`:""}
 <div class="row"><span>목표 칸 관측</span><span>${o.target_covered?"됨":"안 됨 · 재관측 필요"}</span></div>
 ${fp?`<div class="row"><span>관측 범위</span><span>${Math.round(fp.width_m)}m × ${Math.round(fp.height_m)}m (높이 ${Math.round(fp.agl_m)}m)</span></div>`:""}
 ${o.failure_reason?`<div class="row"><span>실패 사유</span><span>${esc(o.failure_reason)}</span></div>`:""}
 <div class="row"><span>출처</span><span>${src}</span></div><div class="row"><span>환경 반영</span><span>${ack}</span></div>
 <div class="row"><span>관측 자원</span><span>${resName(o.resource_id)}</span></div>`;}
function llmBox(g,byId){const l=g.llm;if(!l)return "";
 if(l.status==="OK"){const first=byId[l.order[0]];return `<div class="ai"><b>🤖 AI 추천:</b> ${first?title(first):esc(l.order[0])} 먼저 — ${esc(l.rationale)}</div>`;}
 const r=Array.isArray(l.reason)?"검증 실패":(LLM_REASON[l.reason]||esc(l.reason));return `<div class="ai off">AI 추천 없음 (${r}) · 규칙 순서 사용</div>`;}
async function post(url,body){await fetch(url,{method:"POST",headers:{"Content-Type":"application/json"},body:body?JSON.stringify(body):undefined});load();}
const canSend=t=>(t.purpose_status==="PENDING"||t.purpose_status==="HOLD")&&(t.resume_condition!=="MANUAL_RESOLUTION"||t.hold_reason==="AWAITING_PRIORITY_CHOICE");
async function sendNow(tid){const reason=prompt("지금 보내는 이유를 적어 주세요 (기록에 남습니다)");if(reason)post(`/tasks/${tid}/manual_dispatch`,{reason});}
async function choose(tid){const reason=prompt("먼저 보내는 이유를 적어 주세요 (기록에 남습니다)");if(reason)post("/priority/choose",{order:[tid],reason});}
async function setMode(m){post("/priority/mode",{mode:m,reason:"관제 화면에서 변경"});}
function clockParts(k,s){if(!k)return [mins(s)+" 경과",""];const t=new Date(new Date(k).getTime()+s*1000);
 return [t.toLocaleTimeString("ko-KR",{timeZone:"Asia/Seoul",hour:"2-digit",minute:"2-digit",hour12:false}),t.toLocaleDateString("ko-KR",{timeZone:"Asia/Seoul",year:"numeric",month:"long",day:"numeric"})];}
function clock(k,s){if(!k)return `${mins(s)} 경과`;const t=new Date(new Date(k).getTime()+s*1000);return t.toLocaleString("ko-KR",{timeZone:"Asia/Seoul",month:"numeric",day:"numeric",hour:"2-digit",minute:"2-digit"});}
const wxVals=v=>`${v.wind_ms!=null?v.wind_ms+"m/s ":""}${v.wind_dir_deg!=null?v.wind_dir_deg+"° ":""}${v.temperature_c!=null?v.temperature_c+"℃ ":""}${v.humidity_pct!=null?"습도 "+v.humidity_pct+"%":""}`;
// 바람 화살표: 불어오는 쪽(deg) → 불어가는 쪽(+180) 으로 향함
const arrow=deg=>deg==null?'<span class="mute small">-</span>':`<svg width="26" height="26" viewBox="0 0 26 26" style="transform:rotate(${(deg+180)%360}deg)"><path d="M13 2 L19 14 L14.5 12.5 L14.5 24 L11.5 24 L11.5 12.5 L7 14 Z" fill="#22d3ee"/></svg>`;

function renderTop(b){const k=b.known||{};
 $("mode").innerHTML=b.modes.map(m=>`<button class="${m===b.mode?"on":""}" onclick="setMode('${m}')">${MODE_LABEL[m]}</button>`).join("");
 const [t,d]=clockParts(k.scenario_start_kst,k.simulation_time_s||0);$("clk").textContent=t;$("clkd").textContent=d?`${d} · 시나리오 시각`:"시나리오 시각";
 $("rule").innerHTML=`<b>판단 규칙</b> · AI(${esc(b.llm.model)}): ${b.llm.not_ready_reason?"사용 안 함 — "+esc(LLM_REASON[b.llm.not_ready_reason]||b.llm.not_ready_reason):"사용 중"}. 판단 순서: 인명피해 예상 지역 항상 먼저 → 위험도 → 보호대상까지 거리 → 접수 순서. 위험도 차이가 ${b.similar_delta} 이내이거나 기준이 엇갈리면 한 묶음으로 보여줍니다.`;
 $("ruleShort").textContent=`인명피해 → 위험도 → 보호대상 거리 → 접수 순 · AI ${b.llm.not_ready_reason?"미사용(규칙)":"사용 중"}`;
 $("sub").textContent=`총괄 우선순위 판단 · ${ENV_KO[k.env_source]||k.env_source||"환경 확인 중"}`;
 $("prov").innerHTML=`<b>정보 출처</b> · 진짜 세계: <b>${esc(ENV_KO[k.env_source]||k.env_source||"알 수 없음")}</b> · 위험 칸·확산 예측: <b>${esc(ANALYSIS_KO[k.analysis_source]||k.analysis_source||"없음")}</b>`;
 $("gate").innerHTML=k.run_gate?`<div class="warnbox"><b>주의:</b> 환경의 실행(run)이 바뀌었지만 이전 실행의 기체 점유가 남아 있어 판단을 보류 중입니다 (${esc(k.run_gate)}).</div>`:"";
 // KPI · 경보
 const fires=k.fires||[],tasks=b.auto_order||[];
 const conf=fires.filter(f=>f.status==="CONFIRMED").length,rep=fires.filter(f=>f.status==="REPORTED").length;
 const run=tasks.filter(t=>["IN_EXECUTION","APPROVED","EVALUATING"].includes(t.purpose_status)).length;
 const wait=tasks.filter(t=>["PENDING","HOLD"].includes(t.purpose_status)).length;
 const done=tasks.filter(t=>t.purpose_status==="COMPLETED").length;
 const hum=tasks.filter(t=>t.human_risk===true&&t.purpose_status!=="COMPLETED").length;
 const threats=((b.forecast||{}).threats||[]).length;const decide=(b.groups||[]).some(g=>(g.awaiting||[]).length);
 const K=[["확인된 화재",conf,"곳","#ff4d2e"],["신고 · 확인 전",rep,"건","#ffb020"],["출동 중 임무",run,"건","#ff8a3d"],["대기 · 보류",wait,"건","#8b97a8"],["완료",done,"건","#2fd27c"],["인명 위험 예상",hum+threats,"건","#ff3b3b"]];
 $("kpis").innerHTML=K.map(([l,v,u,c])=>`<div class="kpi" style="--c:${c}"><div class="l">${l}</div><div class="v num" style="color:${v?c:"var(--ink)"}">${v}<span class="s"> ${u}</span></div></div>`).join("");
 let lv=0,ic="✅",msg="이상 없음",sub="신고·확인된 화재 없음";
 if(rep||conf){lv=1;ic="⚠️";msg=conf?`화재 확인 ${conf}곳`:`화재 신고 ${rep}건 — 정찰로 확인 중`;sub=`출동 중 ${run}건 · 대기·보류 ${wait}건 · 완료 ${done}건`;}
 if(conf&&run){lv=2;ic="🔥";msg=`화재 대응 중 — 확인 ${conf}곳 · 출동 ${run}건`;}
 if(hum||threats){lv=3;ic="🚨";msg=`인명 위험 예상 ${hum+threats}건 — 우선 대응`;sub="인명피해 예상 지역은 항상 먼저 판단합니다";}
 if(decide){sub=`지휘 결정 필요: 아래 '지휘 결정'에서 먼저 보낼 곳을 선택하세요 · `+sub;}
 $("alert").className=`alert lv${lv}`;$("alert").innerHTML=`<span class="ic">${ic}</span><span>${msg}</span><small>${esc(sub)}</small>`;
}
function renderGroups(b){const P=$("decideP");const need=(b.groups||[]).some(g=>(g.awaiting||[]).length);P.classList.toggle("decide",need);
 $("groups").innerHTML=b.groups.length?b.groups.map(g=>{
  const st=STATUS_LABEL[g.status]||["상태 확인 중","k"];const auto=g.tasks.find(t=>t.task_id===g.auto_choice);const byId=Object.fromEntries(g.tasks.map(t=>[t.task_id,t]));
  return `<div style="margin-bottom:12px"><div class="ghead"><span class="tag ${st[1]}">${st[0]}</span>${g.kinds.map(k=>`<span class="tag k">${KIND_LABEL[k]||k}</span>`).join("")}${g.status==="AUTO_DECIDED"&&auto?`<span>자동 선택: ${title(auto)}</span>`:""}</div>${llmBox(g,byId)}
  <div class="cards">${g.tasks.map(t=>`<div class="card ${t.task_id===g.auto_choice?"first":""}" style="--cc:${QCOL[t.purpose_status]||"#253041"}">
   <div class="ct"><span>${title(t)}</span>${small(t.task_id)}</div>
   <div class="row"><span>인명피해</span>${human(t.human_risk,t.human_risk_basis)}</div><div class="row"><span>위험도</span><b class="num">${risk(t.risk_score)}</b></div>
   <div class="row"><span>가까운 보호대상</span><span>${near(t)}</span></div><div class="row"><span>상태</span><span>${statusPill(t)}</span></div>
   <div class="row"><span>배정 자원</span><span>${resName(t.resource_id)}</span></div>
   <div class="row"><span>정찰 결과</span>${obsShort(t.observation)}</div>
   ${g.awaiting.includes(t.task_id)?`<button class="go" onclick="choose('${t.task_id}')">이곳 먼저 보내기</button>`:(canSend(t)?`<button class="go" onclick="sendNow('${t.task_id}')">지금 보내기</button>`:"")}</div>`).join("")}</div></div>`}).join(""):'<div class="empty">지금은 사람이 고를 묶음이 없습니다.</div>';}
function renderOrder(b){const o=b.auto_order||[];
 $("order").innerHTML=o.length?o.map((t,i)=>`<div class="q ${i===0?"top1":""}" style="--qc:${QCOL[t.purpose_status]||"#253041"}">
  <div class="rank num">${i+1}</div><div class="task">${title(t)}<span class="tid">${esc(t.task_id)}</span></div>
  <div>${human(t.human_risk,t.human_risk_basis)}</div>
  <div><b class="num">${risk(t.risk_score)}</b>${t.risk_score!=null?`<div class="bar"><i style="width:${Math.max(3,Math.min(100,t.risk_score*100))}%"></i></div>`:""}</div>
  <div class="small">${near(t)}</div><div>${statusPill(t)}</div>
  <div class="small">${resName(t.resource_id)}<br>${obsShort(t.observation)}</div>
  <div>${canSend(t)?`<button class="go" onclick="sendNow('${t.task_id}')">지금 보내기</button>`:""}</div></div>`).join(""):'<div class="pb empty">아직 임무가 없습니다 (신고·임무 접수 전).</div>';}
function renderResults(b){$("results").innerHTML=b.results.length?b.results.map(t=>{const r=(t.observation&&OBS[t.observation.result])||["","warn"];
 const c=r[1]==="fire"?"#ff4d2e":(r[1]==="clear"?"#2fd27c":"#ffb020");return `<div class="card" style="--cc:${c}"><div class="ct"><span>${title(t)}</span>${small(t.task_id)}</div>${obsDetail(t.observation)}</div>`;}).join(""):'<div class="empty">아직 정찰 결과가 없습니다.</div>';}
function renderKnown(k){if(!k){$("fires").innerHTML="";$("wx").innerHTML="";return;}
 $("fires").innerHTML=k.fires.length?k.fires.map(f=>{const s=FIRE_KO[f.status]||[f.status,"warn"];const lp=f.last_partial_observation;
  const part=lp&&f.status!=="NOT_DETECTED_PARTIAL"?`<div class="meta">이후 칸의 ${lp.coverage_fraction!=null?Math.round(lp.coverage_fraction*100)+"%":"일부"}만 봤고 그 범위에는 불 없음 — 나머지는 미확인</div>`:"";
  return `<div class="frow"><span class="fi ${s[1]}"></span><span class="cell num">${esc(f.cell_id)}</span><div><span class="obs ${s[1]}">${s[0]}</span><div class="meta">${esc(SRC_KO[f.source]||f.source)} · ${clock(k.scenario_start_kst,f.sim_time_s)}</div>${part}</div></div>`;}).join(""):'<div class="empty">아직 알고 있는 불이 없습니다.</div>';
 const st=k.stations.length?k.stations.map(o=>{const v=o.values||{};const bad=Object.entries(o.item_status||{}).filter(([,s])=>s.status!=="VALID").map(([i,s])=>`${(WX_LABEL[i]||[i])[0]} ${WX_STATUS[s.status]||s.status}`);
  return `<div class="wx"><div class="compass">${arrow(v.wind_dir_deg)}</div><div><span class="nm">${esc(o.station_name||o.station_id)}</span> <span class="v num">${wxVals(v)||"쓸 수 있는 값 없음"}</span><div class="meta">${esc(o.observed_kst||"")} ${esc(SRC_KO[o.source]||o.source)}${bad.length?" · "+esc(bad.join(", ")):""}</div></div></div>`;}).join(""):'<div class="empty">관측소 관측값이 없습니다.</div>';
 const fw=(k.field_weather||[]).map(o=>`<div class="wx"><div class="compass">${arrow((o.values||{}).wind_dir_deg)}</div><div><span class="nm">${esc(o.cell_id)}</span> <span class="v num">${wxVals(o.values||{})}</span><div class="meta">${clock(k.scenario_start_kst,o.sim_time_s)} ${esc(SRC_KO[o.source]||o.source)}</div></div></div>`).join("");
 const fx=(k.field_weather_excluded||[]).map(o=>`<div class="row"><span>${esc(o.cell_id)}</span><span>${(WX_LABEL[o.item]||[o.item])[0]} — ${WX_STATUS[o.status]||esc(o.status)} <span class="mute small">판단에 쓰지 않음</span></span></div>`).join("");
 const unset=((k.weather_policy||{}).field_policy_unset_items||[]).length;
 $("wx").innerHTML=`<div class="sec">관측소 최신 관측 (그 시각까지)</div>${st}<div class="sec">현장 측정 (지금 유효한 값만)</div>${fw||'<div class="empty">유효한 현장 측정이 없습니다.</div>'}${fx}${unset?'<div class="empty">현장 측정 유효시간이 설정되지 않아 현장 측정값을 판단에 쓰지 않고 관측소 값을 씁니다.</div>':""}`;}
function renderForecast(f){const el=$("forecast");
 if(!f||!f.cell_count){el.innerHTML='<div class="empty">확산 예측이 아직 없습니다.</div>';return;}
 const ref=f.ref||{};$("fcsub").textContent=ref.injected?"현재 시험 주입값 · 관측으로 계산한 예측 아님":"환경 모델 예측";
 const head=`<div class="empty">예측 출처 ${ref.injected?"시험 주입값":"환경 모델"} ${esc(ref.model_version||"")} · 예측 칸 ${f.cell_count}개 · 사전 감시 임무 생성: <b>${f.preemptive_monitor_enabled?"켜짐":"꺼짐 (후보만 표시)"}</b></div>`;
 el.innerHTML=head+(f.threats.length?`<div class="cards">${f.threats.map(t=>`<div class="card" style="--cc:#ff3b3b"><div class="ct"><span>🏘️ ${esc(t.site_id?"보호대상 "+t.site_id:"주거 칸 "+t.key)}</span></div>
  <div class="row"><span>위험</span><span class="human">${t.basis.map(b=>BASIS_KO[b]||b).join(", ")}</span></div>
  <div class="row"><span>예상 도달</span><b class="num">${mins(t.earliest_arrival_s)} 후</b></div>
  <div class="row"><span>해당 칸</span><span>${t.cells.map(esc).join(", ")}</span></div>
  <div class="row"><span>사전 감시</span><span>${t.monitor_task_id?"임무 생성됨"+small(t.monitor_task_id):(f.preemptive_monitor_enabled?"대기":"후보 (생성 꺼짐)")}</span></div></div>`).join("")}</div>`:'<div class="empty">주거지·보호대상에 닿는 예측은 없습니다.</div>');}
async function load(){try{const b=await (await fetch("/priority/board")).json();
  renderTop(b);renderGroups(b);renderOrder(b);renderResults(b);renderForecast(b.forecast);renderKnown(b.known);
  $("live").className="dot";$("livet").textContent="총괄 연결";}
 catch(e){$("live").className="dot off";$("livet").textContent="총괄 연결 끊김";}}
load();setInterval(load,2000);
</script></body></html>
"""
