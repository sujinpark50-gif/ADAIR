(function(){
  var AUTO=false;
  var btn=document.getElementById('auto');
  if(!btn) return;
  btn.onclick=async function(){
    var r=await (await fetch('/api/auto/toggle',{method:'POST'})).json();
    AUTO=r.on;
    btn.textContent=AUTO?'자동 중지':'자동 시작';
    btn.style.background=AUTO?'#c0392b':'#0C8E7E';
  };
  async function tick(){
    if(!AUTO) return;
    try{
      var r=await (await fetch('/api/auto/tick',{method:'POST'})).json();
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
  }
  window.__autoActive=function(){return AUTO;};
  window.__autoDone=function(){ if(AUTO) fetch('/api/auto/done',{method:'POST'}); };
  setInterval(tick,3500);
})();
