#pragma once

static const char dashboardPage[] = R"HTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>小车遥测台</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#0c1820;color:#e9f2f3;font:16px system-ui,sans-serif}
main{max-width:940px;margin:auto;padding:28px 16px}header,article,section{padding:18px;border:1px solid #34474f;border-radius:12px;background:#14252f}
h1{margin:4px 0;font-size:32px}h2{margin:0 0 12px;font-size:18px}small,p,dt{color:#adc2cb}
#grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px;margin:12px 0}
dl{display:grid;grid-template-columns:1fr auto;gap:10px;margin:0}dd{margin:0;font-variant-numeric:tabular-nums}
strong{color:#64e1bd}svg{width:100%;height:125px;background:#0c1820;border-radius:8px}
section{margin:12px 0}p{margin:8px 0;font-size:14px}
</style>
<main><header><small>ESP32-S3 / LIVE TELEMETRY</small><h1>小车遥测台</h1>
<strong id="state">等待车辆数据</strong><p id="link">正在连接…</p></header>
<div id="grid"></div><section><h2>距离趋势 · 最近 30 秒</h2>
<p><span style="color:#65e4c1">● 左 ToF</span>　<span style="color:#f6c477">● 右 ToF</span>　<span style="color:#8db9ff">● 前方超声波</span></p>
<svg id="plot" viewBox="0 0 600 120" preserveAspectRatio="none" aria-label="三路距离趋势"></svg>
<p id="scale">等待距离采样</p></section>
<p>显示控制器缓存。停车后部分读数会保留；缺失读数显示为 —。</p></main>
<script>
const el=id=>document.getElementById(id);
const groups=[
 ['距离传感器',[['left','左 ToF','m'],['right','右 ToF','m'],['front','前方超声波','m']]],
 ['横向控制',[['raw','原始偏差比例',''],['offset','滤波偏差比例',''],['rate','比例变化率','1/s'],['p','比例项','rad'],['d','微分项','rad']]],
 ['转向与动力',[['desired','目标转角','rad'],['applied','应用转角','rad'],['servo','舵机指令','°'],['a','电机 A','PWM'],['b','电机 B','PWM']]]];
for(const [title,rows] of groups){
 const card=document.createElement('article');
 card.innerHTML='<h2>'+title+'</h2><dl>'+rows.map(([key,label])=>'<dt>'+label+'</dt><dd id="'+key+'">—</dd>').join('')+'</dl>';
 el('grid').append(card);
}
const states={0:'已停止',1:'预热中',3:'运行中',4:'障碍停车',5:'等待重新启动',6:'缺少标定',7:'传感器故障',8:'控制时序故障',11:'目标无效',12:'障碍锁定'};
let history=[],previousMs=-1;
function draw(now){
 history=history.filter(s=>now-s.ms<=30000);
 const series=[['left','#65e4c1'],['right','#f6c477'],['front','#8db9ff']];
 const peak=Math.max(0.1,...history.flatMap(s=>series.map(([key])=>s[key]||0)));
 el('scale').textContent='纵轴 0–'+peak.toFixed(2)+' m　·　右端为最新读数';
 el('plot').innerHTML=series.map(([key,color])=>{
   let path='',last=null;
   for(const s of history){
     if(s[key]===null){last=null;continue}
     const x=600-(now-s.ms)/50,y=116-112*s[key]/peak;
     path+=(last===null||s.ms-last>2500?'M':'L')+x+' '+y+' ';
     last=s.ms;
   }
   return '<path fill="none" stroke="'+color+'" stroke-width="2" vector-effect="non-scaling-stroke" d="'+path+'"/>';
 }).join('');
}
async function poll(){
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2500);
 try{
   const res=await fetch('/telemetry',{cache:'no-store',signal:controller.signal});
   if(!res.ok)throw Error();
   const s=await res.json(),live=s.mode==='control'&&s.ms>0;
   el('state').textContent=s.mode==='failed'?'初始化失败 · 已停车':s.mode==='test'?'硬件测试模式':s.ms===0?'正在初始化':states[s.status]||'未知状态';
   const issues=[[1,'ToF 1 异常'],[2,'ToF 2 异常'],[4,'超声波无回波'],[8,'超声波回波无效']].filter(([bit])=>s.issues&bit);
   if(issues.length) el('state').textContent+=' · '+issues.map(([,label])=>label).join('、');
   el('link').textContent='已连接 · 控制器快照 '+(s.ms/1000).toFixed(1)+' s';
   for(const [,rows] of groups)for(const [key,,unit] of rows){
     el(key).textContent=live&&Number.isFinite(s[key])?s[key].toFixed(unit==='PWM'?0:unit==='°'?1:3)+' '+unit:'—';
   }
   if(s.ms<previousMs||!live)history=[];
   if(live&&s.ms!==previousMs)history.push(s);
   previousMs=s.ms;
   draw(s.ms);
 }catch{el('link').textContent='连接中断 · 显示最后收到的数据，正在重连'}
 finally{clearTimeout(timer);setTimeout(poll,500)}
}
poll();
</script></html>)HTML";
