(function(){
  var AUTO=false, BUSY=false, SWITCHING=false;
  var btn=document.getElementById('auto');
  if(!btn) return;
  function paint(){
    btn.textContent=AUTO?'자동 중지':'자동 시작';
    btn.style.background=AUTO?'#c0392b':'#0C8E7E';
  }
  btn.onclick=async function(){
    if(SWITCHING) return;            // 응답 전 연타로 두 번 토글되는 것 방지
    SWITCHING=true;
    if(AUTO){ AUTO=false; paint(); } // 중지는 화면에서 즉시 반영 (다음 tick 을 보내지 않음)
    try{
      var r=await (await fetch('/api/auto/toggle',{method:'POST'})).json();
      AUTO=r.on; paint();
    }finally{ SWITCHING=false; }
  };
  async function tick(){
    if(!AUTO || BUSY) return;         // 이전 tick 이 끝나기 전에는 겹쳐 보내지 않음
    BUSY=true;
    try{
      var r=await (await fetch('/api/auto/tick',{method:'POST'})).json();
      if(!AUTO) return;               // 중지 후 도착한 응답은 화면에 반영하지 않음
      if(r.target){
        var col=document.getElementById('col'), row=document.getElementById('row');
        if(col) col.value=r.target.col;
        if(row) row.value=r.target.row;
        window.tgt={col:r.target.col,row:r.target.row};
      }
      if(r.events && r.events.length){
        var lg=document.getElementById('log'); if(lg) lg.textContent=r.events.join(' | ');
      }
    }catch(e){}
    finally{ BUSY=false; }
  }
  window.__autoActive=function(){return AUTO;};
  window.__autoDone=function(){ if(AUTO) fetch('/api/auto/done',{method:'POST'}); };
  setInterval(tick,3500);
})();
