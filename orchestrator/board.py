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
.human{color:var(--red);font-weight:700}.obs{font-weight:700}.obs.fire{color:var(--red)}.obs.clear{color:var(--green)}.obs.warn{color:var(--amber)}.id{color:var(--mute);font-size:11px;font-weight:400}.ctitle{font-weight:700;margin-bottom:6px}.extra{margin-top:8px;padding-top:6px;border-top:1px dashed var(--line)}.ai{margin:0 0 10px;padding:8px 10px;border-radius:8px;background:color-mix(in srgb,var(--blue) 12%,transparent);font-size:14px}.ai.off{background:transparent;color:var(--mute);padding:0}.row{display:flex;justify-content:space-between;font-size:14px;padding:2px 0}
.row span:first-child{color:var(--mute)}
button.mini{padding:4px 8px;border-radius:6px;border:0;background:var(--red);color:#fff;font-size:12px;cursor:pointer}
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
<h2 style="font-size:16px">총괄이 아는 상황 <small class="id">신고·정찰·관측으로 알게 된 것만 · 시뮬레이션 진짜 상태 아님</small></h2>
<div id="known" style="margin-bottom:16px"></div>
<h2 style="font-size:16px">확산 예측 <small class="id">현재 시험 주입값 · 관측으로 계산한 예측 아님</small></h2>
<div id="forecast" style="margin-bottom:16px"></div>
<h2 style="font-size:16px">정찰·측정 결과</h2>
<div class="cards" id="results" style="margin-bottom:16px"></div>
<h2 style="font-size:16px">전체 판단 순서</h2>
<div class="wrap"><table><thead><tr><th>순서</th><th>임무</th><th>인명피해</th><th>위험도</th><th>가까운 보호대상</th><th>상태</th><th>배정 자원</th><th>정찰 결과</th><th>수동</th></tr></thead><tbody id="order"></tbody></table></div>
</main>
<script>
const MODE_LABEL={AUTO_HIGHER_RISK:"자동 (AI 추천으로 출발)",HUMAN_CHOICE:"수동 (사람이 선택)"};
const KIND_LABEL={SIMILAR:"위험도 비슷",CONFLICT:"기준 엇갈림"};
const STATUS_LABEL={AWAITING_CHOICE:["선택 대기","wait"],PARTIAL_AWAITING_CHOICE:["일부 선택 대기","wait"],AUTO_DECIDED:["자동 출발 (규칙: 위험도 높은 곳 먼저)","auto"],LLM_DECIDED:["자동 출발 (AI 추천, 검증 통과)","auto"],ALL_DISPATCHED:["자원 충분 · 모두 출동","ok"]};
const PURPOSE={PENDING:"대기",HOLD:"보류",EVALUATING:"판단 중",APPROVED:"출동 승인",IN_EXECUTION:"출동 중",COMPLETED:"완료",FAILED:"실패",CANCELLED:"취소"};
const HOLD={AWAITING_PRIORITY_CHOICE:"사람 선택 대기",NO_FEASIBLE_CANDIDATE:"보낼 자원 없음",HUMAN_PRIORITY_CHOSEN:"사람이 선택함",
 EXECUTION_RESPONSE_LOST:"출동 응답 끊김",PROVIDER_LOST_TASK:"기체가 임무 기록 잃음",PROVIDER_OFFLINE:"기체 연락 두절",
 OBSERVATION_REQUIREMENT_NOT_MET:"관측 범위 부족 · 재관측 필요",ENV_APPLY_NOT_ACKED:"환경 반영 미확인",ARRIVED_OBSERVATION_PENDING:"도착 · 관측 대기",
 TARGET_LOCATION_UNKNOWN:"지도에 위치 없음 · 출동 보류",AREA_COVERAGE_INCOMPLETE:"구역 일부만 관측 · 남은 칸 대기",AREA_EVIDENCE_UNSUPPORTED:"구역 전체 관측 불가 (드론 구역 관측 계약 대기)",
 ENV_APPLY_TIMEOUT:"환경 반영 응답 없음",ENV_APPLY_PENDING:"관측 완료 · 환경 반영 확인 대기 (다시 보내지 않음)",ENV_ACK_UNSUPPORTED_FOR_SENSOR:"환경 반영 확인을 지원하지 않는 관측",RESPONSE_VALIDATION_FAILED:"기체 응답 검증 실패",
 RUN_GATE:"실행(run) 전환 대기",RUN_ENDED_BEFORE_OBSERVATION:"실행(run)이 끝나 관측 없음",OPEN_ATTEMPT_EXISTS:"진행 중인 출동 있음",MANUAL_IDLE_CONFIRMED:"기체 정지 확인됨"};
const TASK_KIND={ENV_SENSE:"환경 측정",RECON:"정찰",MONITOR:"감시",RECHECK:"재확인",GROUND_RECON:"지상 확인",GROUND_SUPPORT:"지상 지원"};
const SITE_TYPE={VILLAGE:"마을",FACILITY:"시설",HOUSE:"주택",SCHOOL:"학교",HOSPITAL:"병원",REST_AREA:"휴게소"};
const RES_TYPE={uav:"드론",ugv:"무인차량",fire:"소방차"};
const OBS={MEASURED:["측정 완료","clear"],DETECTED:["불 발견","fire"],NOT_DETECTED:["불 없음","clear"],NO_COVERAGE:["관측 범위 밖","warn"],FAILED:["관측 실패","warn"],ROAD_STATUS:["도로 상황 확인","clear"],NONE:["관측값 없음","warn"]};
const STATE_KO={BURNING:"불타는 중",BURNED:"탄 곳"};
function obsShort(o){if(!o)return '<span class="id">아직 없음</span>';const r=OBS[o.result]||[o.result,"warn"];
 const where=o.detections&&o.detections.length?` (${o.detections.map(d=>esc(d.cell_id)).join(", ")})`:"";return `<span class="obs ${r[1]}">${r[0]}${where}</span>`;}
const COMPLETION_KO={SIMULATION_TEST:"시뮬레이션 시험 완료 (모의 센서·가짜 환경 확인)",INTEGRATED:"실제 연동 완료"};
function completionRow(o){const c=o.completion;return c?`<div class="row"><span>완료 근거</span><span>${COMPLETION_KO[c.evidence_level]||esc(c.evidence_level)}</span></div>`:"";}
function obsDetail(o){return obsDetail0(o)+(o.extras||[]).map(x=>`<div class="extra"><div class="row"><span><b>같은 방문에서 함께 측정</b></span><span></span></div>${obsDetail0(x)}</div>`).join("")+completionRow(o);}
function obsDetail0(o){if(o.sensor_type==="WEATHER"){const v=o.values||{};
  return `<div class="row"><span>결과</span>${obsShort(o)}</div>${Object.keys(WX_LABEL).filter(k=>v[k]!=null).map(k=>`<div class="row"><span>${WX_LABEL[k][0]}</span><span>${v[k]}${WX_LABEL[k][1]}</span></div>`).join("")}
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
const human=(v,basis)=>v===true?`<span class="human">예상됨${basis&&basis.length===1&&basis[0]==="FORECAST"?" (확산 예측)":""}</span>`:(v===false?"없음":"정보 없음");
const mins=s=>s==null?"-":(s<60?`${Math.round(s)}초`:`${Math.round(s/60)}분`);
const WX_LABEL={wind_ms:["풍속","m/s"],wind_dir_deg:["풍향(불어오는 쪽)","°"],temperature_c:["기온","℃"],humidity_pct:["습도","%"]};
const BASIS_KO={FORECAST_REACHES_RESIDENTIAL:"주거지에 도달 예상",FORECAST_REACHES_OCCUPIED_SITE:"사람 있는 보호대상에 도달 예상"};
const risk=v=>v==null?"점수 없음":v.toFixed(2);
const near=t=>t.nearest_protected?`${esc(SITE_TYPE[t.nearest_protected.site_type]||"보호대상")} ${Math.round(t.distance_m)}m${small(t.nearest_protected.site_id)}`:"-";
async function post(url,body){await fetch(url,{method:"POST",headers:{"Content-Type":"application/json"},body:body?JSON.stringify(body):undefined});load();}
const canSend=t=>(t.purpose_status==="PENDING"||t.purpose_status==="HOLD")&&(t.resume_condition!=="MANUAL_RESOLUTION"||t.hold_reason==="AWAITING_PRIORITY_CHOICE");
async function sendNow(tid){const reason=prompt("지금 보내는 이유를 적어 주세요 (기록에 남습니다)");if(reason)post(`/tasks/${tid}/manual_dispatch`,{reason});}
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
   <div class="row"><span>인명피해</span>${human(t.human_risk,t.human_risk_basis)}</div><div class="row"><span>위험도</span><span>${risk(t.risk_score)}</span></div>
   <div class="row"><span>가까운 보호대상</span><span>${near(t)}</span></div><div class="row"><span>상태</span><span>${status(t)}</span></div>
   <div class="row"><span>배정 자원</span><span>${resName(t.resource_id)}</span></div>
   <div class="row"><span>정찰 결과</span>${obsShort(t.observation)}</div>
   ${g.awaiting.includes(t.task_id)?`<button onclick="choose('${t.task_id}')">이곳 먼저 보내기</button>`:(canSend(t)?`<button onclick="sendNow('${t.task_id}')">지금 보내기</button>`:"")}</div>`).join("")}</div></div>`}).join(""):'<p class="note">지금은 사람이 고를 묶음이 없습니다.</p>';
 document.getElementById("order").innerHTML=b.auto_order.map((t,i)=>`<tr><td>${i+1}</td><td>${title(t)}${small(t.task_id)}</td><td>${human(t.human_risk,t.human_risk_basis)}</td><td>${risk(t.risk_score)}</td><td>${near(t)}</td><td>${status(t)}</td><td>${resName(t.resource_id)}</td><td>${obsShort(t.observation)}</td><td>${canSend(t)?`<button class="mini" onclick="sendNow('${t.task_id}')">지금 보내기</button>`:""}</td></tr>`).join("");
 document.getElementById("results").innerHTML=b.results.length?b.results.map(t=>`<div class="card"><div class="ctitle">${title(t)}${small(t.task_id)}</div>${obsDetail(t.observation)}</div>`).join(""):'<p class="note">아직 정찰 결과가 없습니다.</p>';
}
function renderForecast(f){const el=document.getElementById("forecast");
 if(!f||!f.cell_count){el.innerHTML='<p class="note">확산 예측이 아직 없습니다.</p>';return;}
 const ref=f.ref||{};const head=`<p class="note">예측 출처 ${ref.injected?"시험 주입값":"환경 모델"} ${esc(ref.model_version||"")} · 예측 칸 ${f.cell_count}개 · 사전 감시 임무 생성: <b>${f.preemptive_monitor_enabled?"켜짐":"꺼짐 (후보만 표시)"}</b></p>`;
 el.innerHTML=head+(f.threats.length?`<div class="cards">${f.threats.map(t=>`<div class="card"><div class="ctitle">${esc(t.site_id?"보호대상 "+t.site_id:"주거 칸 "+t.key)}</div>
  <div class="row"><span>위험</span><span class="human">${t.basis.map(b=>BASIS_KO[b]||b).join(", ")}</span></div>
  <div class="row"><span>예상 도달</span><span>${mins(t.earliest_arrival_s)} 후</span></div>
  <div class="row"><span>해당 칸</span><span>${t.cells.map(esc).join(", ")}</span></div>
  <div class="row"><span>사전 감시</span><span>${t.monitor_task_id?"임무 생성됨"+small(t.monitor_task_id):(f.preemptive_monitor_enabled?"대기":"후보 (생성 꺼짐)")}</span></div></div>`).join("")}</div>`:'<p class="note">주거지·보호대상에 닿는 예측은 없습니다.</p>');}
const FIRE_KO={REPORTED:["신고됨 (확인 전)","warn"],CONFIRMED:["불 확인","fire"],CONFIRMED_BURNED:["탄 곳 확인","warn"],OBSERVED_CLEAR:["관측 결과 불 없음","clear"],NOT_DETECTED_PARTIAL:["본 범위에 불 없음 (칸 일부만 관측)","warn"]};
const WX_STATUS={EXPIRED:"만료",POLICY_NOT_SET:"유효시간 미설정",OBSERVED_TIME_UNKNOWN:"관측시각 불명",VALUE_INVALID:"값 이상"};
const wxVals=v=>`${v.wind_ms!=null?v.wind_ms+"m/s ":""}${v.wind_dir_deg!=null?v.wind_dir_deg+"° ":""}${v.temperature_c!=null?v.temperature_c+"℃ ":""}${v.humidity_pct!=null?"습도 "+v.humidity_pct+"%":""}`;
const SRC_KO={"119_CALL":"119 신고",EXTERNAL_REPORT:"외부 신고",SIMULATED:"모의 관측",KMA_ASOS_HOURLY:"기상청 관측소(시간자료)",FIXTURE_STATION:"시험 관측소"};
function clock(k,s){if(!k)return `${mins(s)} 경과`;const t=new Date(new Date(k).getTime()+s*1000);return t.toLocaleString("ko-KR",{timeZone:"Asia/Seoul",month:"numeric",day:"numeric",hour:"2-digit",minute:"2-digit"});}
function renderKnown(k){const el=document.getElementById("known");if(!k){el.innerHTML="";return;}
 const fires=k.fires.length?k.fires.map(f=>{const s=FIRE_KO[f.status]||[f.status,"warn"];const lp=f.last_partial_observation;const part=lp&&f.status!=="NOT_DETECTED_PARTIAL"?` <small class="id">이후 칸의 ${lp.coverage_fraction!=null?Math.round(lp.coverage_fraction*100)+"%":"일부"}만 봤고 그 범위에는 불 없음 — 나머지는 미확인</small>`:"";return `<div class="row"><span>${esc(f.cell_id)}</span><span><span class="obs ${s[1]}">${s[0]}</span> · ${esc(SRC_KO[f.source]||f.source)} · ${clock(k.scenario_start_kst,f.sim_time_s)}${part}</span></div>`;}).join(""):'<p class="note">아직 알고 있는 불이 없습니다.</p>';
 const st=k.stations.length?k.stations.map(o=>{const v=o.values||{};const bad=Object.entries(o.item_status||{}).filter(([,s])=>s.status!=="VALID").map(([i,s])=>`${(WX_LABEL[i]||[i])[0]} ${WX_STATUS[s.status]||s.status}`);return `<div class="row"><span>${esc(o.station_name||o.station_id)}</span><span>${wxVals(v)||"쓸 수 있는 값 없음"} <small class="id">${esc(o.observed_kst||"")} ${esc(SRC_KO[o.source]||o.source)}${bad.length?" · "+esc(bad.join(", ")):""}</small></span></div>`;}).join(""):'<p class="note">관측소 관측값이 없습니다.</p>';
 const fw=(k.field_weather||[]).map(o=>`<div class="row"><span>${esc(o.cell_id)}</span><span>${wxVals(o.values||{})} <small class="id">${clock(k.scenario_start_kst,o.sim_time_s)} ${esc(SRC_KO[o.source]||o.source)}</small></span></div>`).join("");
 const fx=(k.field_weather_excluded||[]).map(o=>`<div class="row"><span>${esc(o.cell_id)}</span><span>${(WX_LABEL[o.item]||[o.item])[0]} — ${WX_STATUS[o.status]||esc(o.status)} <small class="id">판단에 쓰지 않음</small></span></div>`).join("");
 const unset=((k.weather_policy||{}).field_policy_unset_items||[]).length;
 const fieldCard=`<div class="card"><div class="ctitle">현장 측정 (지금 유효한 값만)</div>${fw||'<p class="note">유효한 현장 측정이 없습니다.</p>'}${fx}${unset?'<p class="note">현장 측정 유효시간이 설정되지 않아 현장 측정값을 판단에 쓰지 않고 관측소 값을 씁니다.</p>':""}</div>`;
 el.innerHTML=`<p class="note">시나리오 시각: <b>${clock(k.scenario_start_kst,k.simulation_time_s)}</b> · 진짜 세계: 시험용 가짜 환경 (팀 공유 환경 아님) · 위험 칸·확산 예측: <b>${esc(k.analysis_source==="TEST_INJECTED"?"시험 주입값 — 관측으로 계산한 예측 아님 (환경팀 분석 기능 대기)":k.analysis_source||"없음")}</b></p>
 <div class="cards"><div class="card"><div class="ctitle">알고 있는 불</div>${fires}</div><div class="card"><div class="ctitle">관측소 최신 관측 (그 시각까지)</div>${st}</div>${fieldCard}</div>${k.run_gate?`<p class="note"><b>주의:</b> 환경의 실행(run)이 바뀌었지만 이전 실행의 기체 점유가 남아 있어 판단을 보류 중입니다 (${esc(k.run_gate)}).</p>`:""}`;}
const _load=load;load=async function(){await _load();const b=await (await fetch("/priority/board")).json();renderForecast(b.forecast);renderKnown(b.known);};
load();setInterval(load,2000);
</script></body></html>
"""
