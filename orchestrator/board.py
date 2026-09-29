# -*- coding: utf-8 -*-
"""총괄 서버 내장 우선순위 판단 화면 (/board).

기존 웹관제(web/app.py, 통합 담당)는 수정하지 않는다. 통합팀은 같은 데이터를
/priority/board, /priority/mode, /priority/choose 로 가져가 기존 화면에 붙이면 된다.
"""

BOARD_HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>총괄 우선순위 판단</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#1d2330;--mute:#667085;--line:#d9dde5;--red:#c62828;--amber:#b26a00;--blue:#1f5fbf;--green:#2e7d32}
@media (prefers-color-scheme:dark){:root{--bg:#12151b;--card:#1b2029;--ink:#e6e9ef;--mute:#9aa3b2;--line:#2c3340;--red:#ef6b6b;--amber:#f0a741;--blue:#6ea0ff;--green:#6fcf76}}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,"Malgun Gothic",sans-serif;background:var(--bg);color:var(--ink)}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between;padding:16px;border-bottom:1px solid var(--line)}
h1{font-size:18px;margin:0}main{padding:16px;max-width:1200px;margin:auto}
.mode{display:flex;gap:6px}.mode button{border:1px solid var(--line);background:var(--card);color:var(--ink);padding:8px 12px;border-radius:8px;cursor:pointer}
.mode button.on{background:var(--blue);color:#fff;border-color:var(--blue)}
.note{color:var(--mute);font-size:13px;margin:4px 0 16px}
.group{border:1px solid var(--line);border-radius:12px;background:var(--card);padding:12px;margin-bottom:16px}
.ghead{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-bottom:10px;font-weight:600}
.tag{font-size:12px;padding:2px 8px;border-radius:999px;border:1px solid currentColor}
.tag.wait{color:var(--amber)}.tag.auto{color:var(--blue)}.tag.ok{color:var(--green)}.tag.k{color:var(--mute)}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:10px}
.card{border:1px solid var(--line);border-radius:10px;padding:10px}
.card.first{outline:2px solid var(--blue)}
.human{color:var(--red);font-weight:700}.obs{font-weight:700}.obs.fire{color:var(--red)}.obs.clear{color:var(--green)}.obs.warn{color:var(--amber)}.id{color:var(--mute);font-size:11px;font-weight:400}.ctitle{font-weight:700;margin-bottom:6px}.ai{margin:0 0 10px;padding:8px 10px;border-radius:8px;background:color-mix(in srgb,var(--blue) 12%,transparent);font-size:14px}.ai.off{background:transparent;color:var(--mute);padding:0}.row{display:flex;justify-content:space-between;font-size:14px;padding:2px 0}
.row span:first-child{color:var(--mute)}
.card button{margin-top:8px;width:100%;padding:8px;border-radius:8px;border:0;background:var(--red);color:#fff;font-weight:600;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:14px;background:var(--card);border-radius:10px;overflow:hidden}
th,td{padding:8px;border-bottom:1px solid var(--line);text-align:left}th{color:var(--mute);font-weight:500}
.wrap{overflow-x:auto}.act{display:flex;gap:8px}.act button{padding:8px 12px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
</style></head><body>
<header><h1>총괄 우선순위 판단</h1>
<div class="mode" id="mode"></div>
<div class="act"><button onclick="post('/dispatch_pending')">대기 임무 배정 실행</button><button onclick="post('/poll')">진행 갱신</button></div></header>
<main>
<p class="note" id="rule"></p>
<h2 style="font-size:16px">선택 묶음 (비슷하거나 엇갈리는 후보)</h2>
<div id="groups"></div>
<h2 style="font-size:16px">정찰 결과</h2>
<div class="cards" id="results" style="margin-bottom:16px"></div>
<h2 style="font-size:16px">전체 판단 순서</h2>
<div class="wrap"><table><thead><tr><th>순서</th><th>임무</th><th>인명피해</th><th>위험도</th><th>가까운 보호대상</th><th>상태</th><th>배정 자원</th><th>정찰 결과</th></tr></thead><tbody id="order"></tbody></table></div>
</main>
<script>
const MODE_LABEL={HUMAN_CHOICE:"사람이 선택",AUTO_HIGHER_RISK:"자동 (높은 곳 먼저)"};
const KIND_LABEL={SIMILAR:"위험도 비슷",CONFLICT:"기준 엇갈림"};
const STATUS_LABEL={AWAITING_CHOICE:["선택 대기","wait"],PARTIAL_AWAITING_CHOICE:["일부 선택 대기","wait"],AUTO_DECIDED:["자동 결정함 (규칙)","auto"],LLM_DECIDED:["AI 결정함 (검증 통과)","auto"],ALL_DISPATCHED:["자원 충분 · 모두 출동","ok"]};
const PURPOSE={PENDING:"대기",HOLD:"보류",EVALUATING:"판단 중",APPROVED:"출동 승인",IN_EXECUTION:"출동 중",COMPLETED:"완료",FAILED:"실패",CANCELLED:"취소"};
const HOLD={AWAITING_PRIORITY_CHOICE:"사람 선택 대기",NO_FEASIBLE_CANDIDATE:"보낼 자원 없음",HUMAN_PRIORITY_CHOSEN:"사람이 선택함",
 EXECUTION_RESPONSE_LOST:"출동 응답 끊김",PROVIDER_LOST_TASK:"기체가 임무 기록 잃음",PROVIDER_OFFLINE:"기체 연락 두절",
 OBSERVATION_REQUIREMENT_NOT_MET:"관측 범위 부족 · 재관측 필요",ENV_APPLY_NOT_ACKED:"환경 반영 미확인",ARRIVED_OBSERVATION_PENDING:"도착 · 관측 대기"};
const TASK_KIND={RECON:"정찰",MONITOR:"감시",RECHECK:"재확인",GROUND_RECON:"지상 확인",GROUND_SUPPORT:"지상 지원"};
const SITE_TYPE={VILLAGE:"마을",FACILITY:"시설",HOUSE:"주택",SCHOOL:"학교",HOSPITAL:"병원",REST_AREA:"휴게소"};
const RES_TYPE={uav:"드론",ugv:"무인차량",fire:"소방차"};
const OBS={DETECTED:["불 발견","fire"],NOT_DETECTED:["불 없음","clear"],NO_COVERAGE:["관측 범위 밖","warn"],FAILED:["관측 실패","warn"],ROAD_STATUS:["도로 상황 확인","clear"],NONE:["관측값 없음","warn"]};
const STATE_KO={BURNING:"불타는 중",BURNED:"탄 곳"};
function obsShort(o){if(!o)return '<span class="id">아직 없음</span>';const r=OBS[o.result]||[o.result,"warn"];
 const where=o.detections&&o.detections.length?` (${o.detections.map(d=>esc(d.cell_id)).join(", ")})`:"";return `<span class="obs ${r[1]}">${r[0]}${where}</span>`;}
function obsDetail(o){const fp=o.footprint;const src=o.source==="SIMULATED"?"모의 관측 (시험용 센서)":(o.source==="UGV_PROVIDER"||!o.is_fire_observation?"무인차량 도로 관측 (화재 관측 아님)":esc(o.source));
 const ack=o.env_apply?(o.env_apply.result==="ACK"?`확인됨 (환경 버전 ${o.env_apply.state_version})`:(o.env_apply.result==="SKIPPED"?"필요 없음":"확인 안 됨")):"없음";
 return `<div class="row"><span>결과</span>${obsShort(o)}</div>
 ${o.detections&&o.detections.length?`<div class="row"><span>발견한 칸</span><span>${o.detections.map(d=>`${esc(d.cell_id)} ${STATE_KO[d.fire_state]||esc(d.fire_state)}`).join(", ")}</span></div>`:""}
 <div class="row"><span>본 칸</span><span>${o.covered_cells&&o.covered_cells.length?o.covered_cells.map(esc).join(", "):"없음"}</span></div>
 <div class="row"><span>목표 칸 관측</span><span>${o.target_covered?"됨":"안 됨 · 재관측 필요"}</span></div>
 ${fp?`<div class="row"><span>관측 범위</span><span>${Math.round(fp.width_m)}m × ${Math.round(fp.height_m)}m (높이 ${Math.round(fp.agl_m)}m)</span></div>`:""}
 ${o.failure_reason?`<div class="row"><span>실패 사유</span><span>${esc(o.failure_reason)}</span></div>`:""}
 <div class="row"><span>출처</span><span>${src}</span></div><div class="row"><span>환경 반영</span><span>${ack}</span></div>
 <div class="row"><span>관측 자원</span><span>${resName(o.resource_id)}</span></div>`;}
const LLM_REASON={API_KEY_MISSING:"API 키 없음",TIMEOUT_NOT_CONFIGURED:"대기시간 미설정",CALL_LIMIT_NOT_CONFIGURED:"호출 한도 미설정",CALL_LIMIT_REACHED:"호출 한도 도달",NOT_CONNECTED:"연결 안 됨"};
function llmBox(g,byId){const l=g.llm;if(!l)return "";
 if(l.status==="OK"){const first=byId[l.order[0]];return `<div class="ai"><b>AI 추천:</b> ${first?title(first):esc(l.order[0])} 먼저 — ${esc(l.rationale)}</div>`;}
 const r=Array.isArray(l.reason)?"검증 실패":(LLM_REASON[l.reason]||esc(l.reason));return `<div class="ai off">AI 추천 없음 (${r}) · 규칙 순서 사용</div>`;}
const esc=s=>String(s??"").replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const small=s=>s?` <small class="id">${esc(s)}</small>`:"";
function resName(id){if(!id)return "-";const m=/^([A-Z]+)-([a-z]+)(\d+)$/.exec(id);return m?`${m[1]}거점 ${RES_TYPE[m[2]]||m[2]} ${m[3]}호`:esc(id);}
function status(t){if(!t.purpose_status)return "-";let s=PURPOSE[t.purpose_status]||t.purpose_status;
 if(t.hold_reason){const h=t.hold_reason.split(":")[0];s+=` (${HOLD[h]||(h.startsWith("HANDOVER")?"인계 필요":"사유 기록됨")})`;}return esc(s);}
const title=t=>`${esc(TASK_KIND[t.kind]||"임무")} · ${esc(t.cell_id||"-")}`;
const human=v=>v===true?'<span class="human">예상됨</span>':(v===false?"없음":"정보 없음");
const risk=v=>v==null?"점수 없음":v.toFixed(2);
const near=t=>t.nearest_protected?`${esc(SITE_TYPE[t.nearest_protected.site_type]||"보호대상")} ${Math.round(t.distance_m)}m${small(t.nearest_protected.site_id)}`:"-";
async function post(url,body){await fetch(url,{method:"POST",headers:{"Content-Type":"application/json"},body:body?JSON.stringify(body):undefined});load();}
async function choose(tid){const reason=prompt("먼저 보내는 이유를 적어 주세요 (기록에 남습니다)");if(reason)post("/priority/choose",{order:[tid],reason});}
async function setMode(m){post("/priority/mode",{mode:m,reason:"관제 화면에서 변경"});}
async function load(){
 const b=await (await fetch("/priority/board")).json();
 document.getElementById("mode").innerHTML=b.modes.map(m=>`<button class="${m===b.mode?"on":""}" onclick="setMode('${m}')">${MODE_LABEL[m]}</button>`).join("");
 document.getElementById("rule").textContent=`AI(${b.llm.model}): ${b.llm.not_ready_reason?"사용 안 함 — "+(LLM_REASON[b.llm.not_ready_reason]||b.llm.not_ready_reason):"사용 중"}. 판단 순서: 인명피해 예상 지역 항상 먼저 → 위험도 → 보호대상까지 거리 → 접수 순서. 위험도 차이가 ${b.similar_delta} 이내이거나 기준이 엇갈리면 한 묶음으로 보여줍니다.`;
 document.getElementById("groups").innerHTML=b.groups.length?b.groups.map(g=>{
  const st=STATUS_LABEL[g.status]||["상태 확인 중","k"];const auto=g.tasks.find(t=>t.task_id===g.auto_choice);const byId=Object.fromEntries(g.tasks.map(t=>[t.task_id,t]));
  return `<div class="group"><div class="ghead"><span class="tag ${st[1]}">${st[0]}</span>${g.kinds.map(k=>`<span class="tag k">${KIND_LABEL[k]||k}</span>`).join("")}${g.status==="AUTO_DECIDED"&&auto?`<span>자동 선택: ${title(auto)}</span>`:""}</div>${llmBox(g,byId)}
  <div class="cards">${g.tasks.map(t=>`<div class="card ${t.task_id===g.auto_choice?"first":""}">
   <div class="ctitle">${title(t)}${small(t.task_id)}</div>
   <div class="row"><span>인명피해</span>${human(t.human_risk)}</div><div class="row"><span>위험도</span><span>${risk(t.risk_score)}</span></div>
   <div class="row"><span>가까운 보호대상</span><span>${near(t)}</span></div><div class="row"><span>상태</span><span>${status(t)}</span></div>
   <div class="row"><span>배정 자원</span><span>${resName(t.resource_id)}</span></div>
   <div class="row"><span>정찰 결과</span>${obsShort(t.observation)}</div>
   ${g.awaiting.includes(t.task_id)?`<button onclick="choose('${t.task_id}')">이곳 먼저 보내기</button>`:""}</div>`).join("")}</div></div>`}).join(""):'<p class="note">지금은 사람이 고를 묶음이 없습니다.</p>';
 document.getElementById("order").innerHTML=b.auto_order.map((t,i)=>`<tr><td>${i+1}</td><td>${title(t)}${small(t.task_id)}</td><td>${human(t.human_risk)}</td><td>${risk(t.risk_score)}</td><td>${near(t)}</td><td>${status(t)}</td><td>${resName(t.resource_id)}</td><td>${obsShort(t.observation)}</td></tr>`).join("");
 document.getElementById("results").innerHTML=b.results.length?b.results.map(t=>`<div class="card"><div class="ctitle">${title(t)}${small(t.task_id)}</div>${obsDetail(t.observation)}</div>`).join(""):'<p class="note">아직 정찰 결과가 없습니다.</p>';
}
load();setInterval(load,2000);
</script></body></html>
"""
