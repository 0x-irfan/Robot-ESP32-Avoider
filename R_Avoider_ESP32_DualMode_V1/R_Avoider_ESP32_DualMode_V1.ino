#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char* AP_SSID = "ESP32-Robot";
const char* AP_PASS = "12345678";         
IPAddress apIP(192, 168, 4, 1);         
IPAddress apGW(192, 168, 4, 1);
IPAddress apMask(255, 255, 255, 0);

#define SERVO_PIN 13
#define TRIG_PIN  33
#define ECHO_PIN  34

#define PWMA 18
#define AIN1 23
#define AIN2 19
#define PWMB 14
#define BIN1 26
#define BIN2 25
#define STBY 27

const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = true;

#define CH_A 4
#define CH_B 5

WebServer server(80);
Servo servo;

char  motion      = 'S';       
int   servoAngle  = 90;        
int   speedVal    = 200;       
float distanceCm  = -1;       

unsigned long lastCmd     = 0;
unsigned long lastSensor  = 0;

bool autoMode = false;         // false = MANUAL (web), true = AUTONOMOUS

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Robot Avoider</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@500;700;900&family=Share+Tech+Mono&display=swap');
:root{
  --bg:#020205; --pink:#ff0055; --cyan:#00f0ff; --cyanMuted:#008b99;
  --yellow:#ffe600; --red:#ff1e56; --green:#00ff66;
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;margin:0;padding:0;}
html,body{height:100%;}
body{
  font-family:'Share TechMono',monospace,monospace;
  background-color:var(--bg);
  background-image:
    radial-gradient(ellipse 55% 45% at 12% 15%, rgba(255,0,85,.20) 0%, transparent 62%),
    radial-gradient(ellipse 55% 45% at 88% 85%, rgba(0,240,255,.17) 0%, transparent 62%),
    radial-gradient(ellipse 40% 35% at 50% 100%, rgba(160,0,255,.14) 0%, transparent 65%),
    radial-gradient(circle at 50% 50%, rgba(20,0,40,0.85) 0%, rgba(2,2,5,1) 100%),
    linear-gradient(rgba(0,240,255,.07) 1px,transparent 1px),
    linear-gradient(90deg, rgba(255,0,85,.07) 1px, transparent 1px);
  background-size:140% 140%, 140% 140%, 100% 100%, 100% 100%, 26px 26px, 26px 26px;
  background-position:0% 0%, 100% 100%, 50% 100%, 50% 50%, 0 0, 0 0;
  animation:bgDrift 45s ease-in-out infinite;
  box-shadow:
    inset 0 0 170px 50px rgba(0,0,0,.8),
    inset 0 0 0 2px rgba(0,240,255,.6),
    inset 0 0 22px 3px rgba(0,240,255,.45),
    inset 0 0 55px 10px rgba(0,240,255,.2),
    inset 0 0 90px 18px rgba(255,0,85,.16);
  color:#cbd5e1;
  position:relative;isolation:isolate;
  user-select:none;-webkit-user-select:none;touch-action:manipulation;
  width:100vw;height:100vh;height:100dvh;overflow:hidden;
  display:flex;flex-direction:column;
  padding:8px;
}
body::after{
  content:'';position:fixed;inset:0;z-index:80;pointer-events:none;
  background:repeating-linear-gradient(to bottom,rgba(255,255,255,.045) 0px,rgba(255,255,255,.045) 1px,transparent 1px,transparent 3px);
  mix-blend-mode:overlay;
  animation:scanMove 8s linear infinite;
}
@keyframes bgDrift{
  0%{background-position:0% 0%, 100% 100%, 50% 100%, 50% 50%, 0 0, 0 0;}
  50%{background-position:5% 8%, 95% 92%, 46% 96%, 50% 50%, 60px 40px, 40px 60px;}
  100%{background-position:0% 0%, 100% 100%, 50% 100%, 50% 50%, 120px 80px, 80px 120px;}
}
@keyframes scanMove{from{background-position-y:0;}to{background-position-y:120px;}}
.orb{font-family:'Orbitron',sans-serif;}

/* ---------- header ---------- */
header{height:32px;flex-shrink:0;display:flex;align-items:center;justify-content:flex-end;padding:0 4px;margin-bottom:6px;gap:8px;border-bottom:1px solid rgba(0,240,255,0.35);box-shadow:0 4px 16px rgba(255,0,85,.12),0 4px 12px rgba(0,0,0,0.5);position:relative;z-index:2;}
.brand{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);display:flex;align-items:center;gap:6px;font-weight:900;letter-spacing:.15em;font-size:1.1rem;}
.brand .r{color:var(--cyan);text-shadow:0 0 10px var(--cyan),-2px 0 var(--pink);}
.brand .a{color:var(--yellow);text-shadow:0 0 10px var(--yellow);margin-left:4px;}
.hdr-right{display:flex;align-items:center;gap:8px;flex-shrink:0;}
.online{border:1px solid var(--cyan);background:rgba(0,240,255,0.05);display:flex;align-items:center;gap:6px;padding:3px 10px;font-size:.58rem;font-weight:700;letter-spacing:.1em;box-shadow:0 0 8px rgba(0,240,255,0.2);}
.online .dot{width:8px;height:8px;background:var(--red);box-shadow:0 0 8px var(--red);display:inline-block;border-radius:1px;}
.online.on .dot{background:var(--green);box-shadow:0 0 8px var(--green);}

/* ---------- main 3-column grid ---------- */
main{flex:1;display:grid;grid-template-columns:repeat(3,1fr);gap:8px;min-height:0;overflow:hidden;}
.col{display:flex;flex-direction:column;gap:8px;min-height:0;}
.hud{background:rgba(6,12,19,0.85);border:1.4px solid var(--cyan);box-shadow:inset 0 0 14px rgba(0,240,255,.1),0 0 14px rgba(0,240,255,.22);backdrop-filter:blur(5px);position:relative;overflow:hidden;}
.hud::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,rgba(0,240,255,.9),transparent);opacity:.6;}
.ttl{color:var(--cyan);font-weight:700;font-size:.7rem;letter-spacing:.12em;text-shadow:0 0 5px rgba(0,240,255,0.4);}

/* ---------- servo panel ---------- */
.p-servo{flex:1.4;padding:8px 10px;display:flex;flex-direction:column;justify-content:space-between;min-height:0;}
.p-servo .hd{display:flex;align-items:center;justify-content:space-between;font-size:.7rem;font-weight:700;flex-shrink:0;}
.p-servo .hd b{color:var(--yellow);}
.gauge-wrap{flex:1;display:flex;align-items:center;justify-content:center;margin:auto 0;width:100%;min-height:0;}
.gauge-svg{width:min(210px,92%);height:auto;max-height:100%;overflow:visible;}
.gauge-track{fill:none;stroke:url(#gaugeTrack);stroke-width:3;stroke-linecap:round;opacity:.55;}
.gauge-tick-major{stroke:var(--cyan);stroke-width:2;filter:drop-shadow(0 0 3px var(--cyan));}
.gauge-tick-minor{stroke:rgba(0,240,255,.4);stroke-width:1;}
.gauge-label{fill:#9fb4c2;font-size:9px;font-family:'Share Tech Mono',monospace;text-anchor:middle;}
.gauge-label.mid{fill:var(--yellow);font-weight:700;}
.gauge-needle-group{transition:transform .25s cubic-bezier(.34,1.56,.64,1);transform-origin:100px 100px;}
.gauge-needle{stroke:var(--yellow);stroke-width:2.6;stroke-linecap:round;filter:drop-shadow(0 0 6px var(--yellow));}
.gauge-needle-tip{fill:var(--yellow);filter:drop-shadow(0 0 6px var(--yellow));}
.gauge-hub-ring{fill:#04080d;stroke:var(--yellow);stroke-width:2;filter:drop-shadow(0 0 4px var(--yellow));}
.gauge-hub{fill:var(--yellow);filter:drop-shadow(0 0 5px var(--yellow));}
.needle{width:3px;height:70%;background:var(--yellow);position:absolute;bottom:2px;transform-origin:bottom center;transition:transform .25s;box-shadow:0 0 10px var(--yellow);}
.hub{width:14px;height:14px;border-radius:50%;background:var(--yellow);border:2px solid #000;position:absolute;bottom:-4px;box-shadow:0 0 8px var(--yellow);z-index:2;}
.presets{display:grid;grid-template-columns:repeat(7,1fr);gap:3px;font-size:.6rem;font-weight:700;flex-shrink:0;}
.presets button{border:1px solid rgba(0,240,255,.5);background:transparent;color:#cbd5e1;padding:4px 0;cursor:pointer;font-family:inherit;}
.presets button.on{background:var(--cyan);color:#000;box-shadow:0 0 10px var(--cyan);font-weight:800;border-color:transparent;}

/* ---------- terminal panel ---------- */
.p-term{flex:1;padding:6px 8px;display:flex;flex-direction:column;min-height:0;}
.term-hd{display:flex;align-items:center;justify-content:space-between;margin-bottom:5px;flex-shrink:0;}
.term-hd .btns{display:flex;gap:6px;}
.term-hd button{border:1px solid var(--pink);color:var(--pink);background:transparent;font-size:.56rem;font-weight:700;padding:2px 7px;letter-spacing:.08em;cursor:pointer;font-family:inherit;}
.term-hd button:active{background:rgba(255,0,85,.25);}
#term{flex:1;border:1px solid rgba(0,240,255,.6);background:#020508;padding:5px;font-size:.64rem;line-height:1.45;overflow-y:auto;user-select:text;-webkit-user-select:text;}
#term div{color:var(--green);}
#term .t{color:var(--cyanMuted);}
#term .cm{color:var(--yellow);}
#term .sv{color:var(--pink);}
#term .er{color:var(--red);}

/* ---------- network stats ---------- */
.p-net{padding:6px 8px;flex-shrink:0;}
.net-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;text-align:center;}
.net-cell{background:#04080d;border:1.1px solid var(--cyan);box-shadow:inset 0 0 10px rgba(0,240,255,.05),0 0 8px rgba(0,240,255,.12);padding:4px 2px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;}
.net-cell .lbl{display:flex;align-items:center;gap:3px;font-size:.56rem;color:#94a3b8;font-weight:700;}
.net-cell .lbl svg{width:11px;height:11px;flex-shrink:0;}
.net-cell b{font-size:.7rem;letter-spacing:.05em;color:var(--cyan);font-weight:800;}
.lat-row{display:flex;align-items:center;gap:4px;}
.lat-row b{color:var(--green);transition:color .2s;}
.lat-row .unit{font-size:.52rem;color:inherit;}
.lat-spark{width:26px;height:12px;flex-shrink:0;overflow:visible;}

/* ---------- sonar / radar (now centered in the middle column) ---------- */
.p-sonar{padding:6px 8px;flex:1;display:flex;flex-direction:column;min-height:0;}
.p-sonar .hd{display:flex;align-items:center;justify-content:space-between;font-size:.7rem;font-weight:700;flex-shrink:0;}
.radar-wrap{flex:1;display:flex;align-items:center;justify-content:center;min-height:0;position:relative;}
.radar-svg{width:100%;height:100%;max-height:100%;overflow:visible;}
.radar-ring{fill:none;stroke:rgba(0,240,255,.35);stroke-width:1;}
.radar-ring.outer{stroke:rgba(0,240,255,.55);}
.radar-cross{stroke:rgba(0,240,255,.25);stroke-width:1;}
.radar-tick{fill:#5b7c8c;font-size:6px;font-family:'Share Tech Mono',monospace;}
.radar-range-lbl{fill:#3d5866;font-size:5.5px;font-family:'Share Tech Mono',monospace;}
.radar-sweep-group{transition:transform .22s ease-out;transform-origin:100px 100px;}
.radar-sweep{stroke:var(--green);stroke-width:1.6;filter:drop-shadow(0 0 4px var(--green));}
.radar-sweep-fan{fill:var(--green);opacity:.16;}
.radar-blip-group{transition:opacity .2s;}
.radar-blip{fill:var(--red);filter:drop-shadow(0 0 6px var(--red));transition:cy .22s ease-out;}
.radar-blip-ring{fill:none;stroke:var(--red);stroke-width:1;opacity:.6;transition:cy .22s ease-out;animation:radarPing 1.4s ease-out infinite;}
@keyframes radarPing{0%{opacity:.7;} 70%{opacity:0;} 100%{opacity:0;}}
.radar-readout{display:flex;align-items:baseline;justify-content:center;gap:5px;margin-top:2px;flex-shrink:0;}
.radar-readout b{font-family:'Orbitron',sans-serif;color:var(--red);font-weight:800;font-size:1.45rem;letter-spacing:-.02em;text-shadow:0 0 12px var(--red);}
.radar-readout span{color:var(--cyan);font-weight:700;font-size:.62rem;letter-spacing:.1em;}

/* ---------- control override (right column) ---------- */
.p-ctrl{padding:8px;display:flex;flex-direction:column;justify-content:space-between;min-height:0;}
.ctrl-hd{display:flex;align-items:center;justify-content:space-between;font-size:.68rem;font-weight:700;flex-shrink:0;flex-wrap:wrap;gap:4px;}
.ctrl-hd .l{display:flex;align-items:center;gap:6px;}
.ctrl-hd .tag{font-size:.52rem;color:var(--pink);border:1px solid rgba(255,0,85,.5);padding:1px 5px;font-weight:700;}
.ctrl-hd .r{display:flex;gap:8px;font-size:.58rem;color:#94a3b8;}
.ctrl-hd .r b{color:var(--yellow);font-weight:700;margin-left:2px;}
.ctrl-hd .r .cy{color:var(--cyan);}

.stick-area{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;margin:2px 0;position:relative;min-height:0;}
.vec-row{display:flex;align-items:center;justify-content:center;gap:10px;font-size:.58rem;color:#94a3b8;margin-bottom:5px;flex-shrink:0;}
.vec-row .cy{color:var(--cyan);}
.vec-row .dot-sep{color:var(--pink);}
.vec-row .yl{color:var(--yellow);}

.joy-base{position:relative;width:clamp(145px,40vh,235px);height:clamp(145px,40vh,235px);border-radius:50%;border:2px solid rgba(255,0,85,.6);background:rgba(4,8,13,.95);display:flex;align-items:center;justify-content:center;box-shadow:0 0 15px rgba(255,0,85,.25);touch-action:none;cursor:pointer;flex-shrink:0;}
.joy-base.active{border-color:var(--cyan);box-shadow:0 0 22px rgba(0,240,255,.65),inset 0 0 20px rgba(255,0,85,.25);}
.ring{position:absolute;border-radius:50%;pointer-events:none;}
.ring1{inset:4px;border:1px dashed rgba(0,240,255,.3);}
.ring2{width:60%;height:60%;border:1px solid rgba(0,240,255,.2);}
.ring3{width:36%;height:36%;border:1px dashed rgba(255,0,85,.3);}
.axis-h{position:absolute;width:100%;height:1px;background:linear-gradient(90deg,transparent,rgba(0,240,255,.4),transparent);pointer-events:none;}
.axis-v{position:absolute;height:100%;width:1px;background:linear-gradient(180deg,transparent,rgba(0,240,255,.4),transparent);pointer-events:none;}
.diag1{position:absolute;width:100%;height:1px;background:rgba(0,240,255,.1);transform:rotate(45deg);pointer-events:none;}
.diag2{position:absolute;width:100%;height:1px;background:rgba(0,240,255,.1);transform:rotate(-45deg);pointer-events:none;}
.dirlbl{position:absolute;font-size:.52rem;font-weight:700;color:rgba(0,240,255,.8);display:flex;flex-direction:column;align-items:center;gap:1px;pointer-events:none;}
.dirlbl svg{width:10px;height:10px;}
.dl-up{top:4px;}
.dl-down{bottom:4px;}
.dl-left{left:5px;flex-direction:row;}
.dl-right{right:5px;flex-direction:row;}

.thumb{position:absolute;width:clamp(52px,17vh,78px);height:clamp(52px,17vh,78px);border-radius:50%;background:linear-gradient(180deg,#091824,#04080e);border:2px solid var(--cyan);box-shadow:0 0 14px rgba(0,240,255,.7),inset 0 0 10px rgba(0,240,255,.3);display:flex;align-items:center;justify-content:center;cursor:grab;touch-action:none;z-index:2;will-change:transform;}
.thumb.active{border-color:var(--yellow);box-shadow:0 0 20px var(--cyan),0 0 25px rgba(255,230,0,.6);}
.thumb .in1{width:56%;height:56%;border-radius:50%;border:1px solid rgba(0,240,255,.5);background:#07131b;box-shadow:inset 0 0 6px var(--cyan);display:flex;align-items:center;justify-content:center;}
.thumb .in2{width:44%;height:44%;border-radius:50%;background:var(--pink);border:1px solid var(--yellow);box-shadow:0 0 8px var(--pink);display:flex;align-items:center;justify-content:center;}
.thumb .in3{width:38%;height:38%;border-radius:50%;background:var(--yellow);box-shadow:0 0 4px var(--yellow);}
.notch{position:absolute;width:4px;height:4px;border-radius:50%;background:rgba(0,240,255,.7);}
.n-top{top:3px;}.n-bottom{bottom:3px;}.n-left{left:3px;}.n-right{right:3px;}

.state-row{display:flex;align-items:center;justify-content:center;gap:6px;font-size:.58rem;letter-spacing:.08em;text-transform:uppercase;flex-shrink:0;margin-top:4px;}
.state-row .lbl{color:#94a3b8;}
.state-badge{color:var(--cyan);font-weight:800;padding:2px 8px;border:1px solid rgba(0,240,255,.4);background:#020508;}
.state-badge.act{color:var(--yellow);border-color:var(--yellow);background:#020508;box-shadow:0 0 8px rgba(255,230,0,.5);}

.throttle-row{background:rgba(4,8,13,.8);border:1.1px solid var(--cyan);box-shadow:inset 0 0 10px rgba(0,240,255,.05),0 0 8px rgba(0,240,255,.12);padding:5px 8px;display:flex;align-items:center;gap:8px;flex-shrink:0;}
.throttle-row label{color:var(--cyan);font-weight:700;font-size:.62rem;letter-spacing:.08em;white-space:nowrap;}
input[type=range]{-webkit-appearance:none;appearance:none;flex:1;height:8px;background:#101c27;border:1px solid #1f364d;border-radius:9999px;}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;height:18px;width:18px;border-radius:50%;background:var(--red);cursor:pointer;box-shadow:0 0 10px var(--red);border:2px solid #fff;}
input[type=range]::-moz-range-thumb{height:18px;width:18px;border-radius:50%;background:var(--red);cursor:pointer;box-shadow:0 0 10px var(--red);border:2px solid #fff;}
#throttleValue{color:var(--red);font-weight:800;font-size:.82rem;min-width:28px;text-align:right;text-shadow:0 0 6px var(--red);}

::-webkit-scrollbar{width:4px;}
::-webkit-scrollbar-track{background:#060b11;}
::-webkit-scrollbar-thumb{background:var(--cyan);}

/* ---------- tombol mode autonomous (kiri atas) ---------- */
.mode-btn{margin-right:auto;display:flex;align-items:center;gap:6px;padding:3px 10px;font-family:'Orbitron',sans-serif;font-size:.56rem;font-weight:700;letter-spacing:.1em;color:var(--pink);background:rgba(255,0,85,0.06);border:1px solid var(--pink);box-shadow:0 0 8px rgba(255,0,85,0.25);cursor:pointer;position:relative;z-index:3;}
.mode-btn .dot{width:8px;height:8px;background:var(--red);box-shadow:0 0 8px var(--red);display:inline-block;border-radius:1px;}
.mode-btn:active{background:rgba(255,0,85,.25);}
.mode-btn.on{color:var(--green);background:rgba(0,255,102,0.08);border-color:var(--green);box-shadow:0 0 12px rgba(0,255,102,0.45);}
.mode-btn.on .dot{background:var(--green);box-shadow:0 0 8px var(--green);}
body.auto .joy-base,body.auto .presets,body.auto .throttle-row{pointer-events:none;opacity:.35;}

/* ---------- portrait fallback: stack columns, allow scroll ---------- */
@media (orientation:portrait){
  body{height:auto;min-height:100vh;min-height:100dvh;overflow-y:auto;overflow-x:hidden;}
  main{display:flex;flex-direction:column;overflow:visible;}
  .col{flex:none;}
  .p-servo,.p-term,.p-net,.p-sonar,.p-ctrl{flex:none;}
  .p-sonar{min-height:220px;}
  .joy-base{width:clamp(195px,68vw,255px);height:clamp(195px,68vw,255px);}
  .thumb{width:clamp(66px,23vw,86px);height:clamp(66px,23vw,86px);}
  #term{min-height:100px;}
}
</style>
</head>
<body>

<header>
  <button class="mode-btn" id="modeBtn" type="button"><i class="dot"></i><span>AUTONOMOUS</span></button>
  <div class="brand orb"><span class="r">ROBOT</span><span class="a">AVOIDER</span></div>
  <div class="hdr-right">
    <div class="online" id="conn"><i class="dot"></i><span id="connText">OFFLINE</span></div>
  </div>
</header>

<main>
  <!-- ===================== LEFT COLUMN: servo + terminal ===================== -->
  <section class="col">
    <div class="hud p-servo">
      <div class="hd"><span class="ttl">SERVO SG90</span><span><b id="servoHeaderAngle">90</b>&deg;</span></div>
      <div class="gauge-wrap">
        <svg class="gauge-svg" viewBox="0 0 200 110">
          <defs>
            <linearGradient id="gaugeTrack" x1="0" y1="0" x2="1" y2="0">
              <stop offset="0%" stop-color="var(--pink)"></stop>
              <stop offset="50%" stop-color="var(--cyan)"></stop>
              <stop offset="100%" stop-color="var(--pink)"></stop>
            </linearGradient>
          </defs>
          <path class="gauge-track" d="M 8 100 A 92 92 0 0 1 192 100"></path>

          <!-- minor ticks (every 10 deg, skipping majors) -->
          <line class="gauge-tick-minor" x1="9.4" y1="84.0" x2="16.3" y2="85.2"></line>
          <line class="gauge-tick-minor" x1="13.6" y1="68.5" x2="20.1" y2="70.9"></line>
          <line class="gauge-tick-minor" x1="29.5" y1="40.9" x2="34.9" y2="45.4"></line>
          <line class="gauge-tick-minor" x1="40.9" y1="29.5" x2="45.4" y2="34.9"></line>
          <line class="gauge-tick-minor" x1="68.5" y1="13.6" x2="70.9" y2="20.1"></line>
          <line class="gauge-tick-minor" x1="84.0" y1="9.4" x2="85.2" y2="16.3"></line>
          <line class="gauge-tick-minor" x1="116.0" y1="9.4" x2="114.8" y2="16.3"></line>
          <line class="gauge-tick-minor" x1="131.5" y1="13.6" x2="129.1" y2="20.1"></line>
          <line class="gauge-tick-minor" x1="159.1" y1="29.5" x2="154.6" y2="34.9"></line>
          <line class="gauge-tick-minor" x1="170.5" y1="40.9" x2="165.1" y2="45.4"></line>
          <line class="gauge-tick-minor" x1="186.5" y1="68.5" x2="179.9" y2="70.9"></line>
          <line class="gauge-tick-minor" x1="190.6" y1="84.0" x2="183.7" y2="85.2"></line>

          <!-- major ticks + degree labels, every 30 deg -->
          <line class="gauge-tick-major" x1="8" y1="100" x2="22" y2="100"></line>
          <text class="gauge-label" x="36" y="103">0&deg;</text>
          <line class="gauge-tick-major" x1="20.3" y1="54" x2="32.5" y2="61"></line>
          <text class="gauge-label" x="44.6" y="68">30&deg;</text>
          <line class="gauge-tick-major" x1="54" y1="20.3" x2="61" y2="32.5"></line>
          <text class="gauge-label" x="68" y="44.6">60&deg;</text>
          <line class="gauge-tick-major" x1="100" y1="8" x2="100" y2="22"></line>
          <text class="gauge-label mid" x="100" y="34">90&deg;</text>
          <line class="gauge-tick-major" x1="146" y1="20.3" x2="139" y2="32.5"></line>
          <text class="gauge-label" x="132" y="44.6">120&deg;</text>
          <line class="gauge-tick-major" x1="179.7" y1="54" x2="167.6" y2="61"></line>
          <text class="gauge-label" x="155.4" y="68">150&deg;</text>
          <line class="gauge-tick-major" x1="192" y1="100" x2="178" y2="100"></line>
          <text class="gauge-label" x="164" y="103">180&deg;</text>

          <g class="gauge-needle-group" id="servoNeedleGroup">
            <line class="gauge-needle" x1="100" y1="100" x2="100" y2="26"></line>
            <polygon class="gauge-needle-tip" points="100,17 95.5,28 104.5,28"></polygon>
          </g>
          <circle class="gauge-hub-ring" cx="100" cy="100" r="10"></circle>
          <circle class="gauge-hub" cx="100" cy="100" r="4.5"></circle>
        </svg>
      </div>
      <div class="presets" id="presets">
        <button data-angle="0">0</button>
        <button data-angle="30">30</button>
        <button data-angle="60">60</button>
        <button data-angle="90" class="on">90</button>
        <button data-angle="120">120</button>
        <button data-angle="150">150</button>
        <button data-angle="180">180</button>
      </div>
    </div>

    <div class="hud p-term">
      <div class="term-hd">
        <span class="ttl">COM.TERMINAL</span>
        <div class="btns">
          <button id="btnTerminalPause">PAUSE</button>
          <button id="btnTerminalClear">CLEAR</button>
        </div>
      </div>
      <div id="term"></div>
    </div>
  </section>

  <!-- ===================== CENTER COLUMN: telemetry ===================== -->
  <section class="col">
    <div class="hud p-net">
      <div class="net-grid">
        <div class="net-cell">
          <div class="lbl">
            <svg viewBox="0 0 24 24" fill="none" stroke="var(--cyan)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M8.111 16.404a5.5 5.5 0 017.778 0M12 20h.01m-7.08-7.071c3.904-3.905 10.236-3.905 14.141 0M1.394 9.393c5.857-5.857 15.355-5.857 21.213 0"></path></svg>
            <span>TX SIGNAL</span>
          </div>
          <b>BCAST</b>
        </div>
        <div class="net-cell">
          <div class="lbl">
            <svg viewBox="0 0 24 24" fill="none" stroke="var(--pink)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M13 10V3L4 14h7v7l9-11h-7z"></path></svg>
            <span>FREQ CH</span>
          </div>
          <b id="wf-ch">--</b>
        </div>
        <div class="net-cell">
          <div class="lbl">
            <svg viewBox="0 0 24 24" fill="none" stroke="var(--yellow)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z"></path></svg>
            <span>LATENCY</span>
          </div>
          <div class="lat-row">
            <b id="wf-lat">--<span class="unit">ms</span></b>
            <svg class="lat-spark" viewBox="0 0 40 16" preserveAspectRatio="none">
              <polyline id="latPoly" points="" fill="none" stroke="var(--green)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"></polyline>
            </svg>
          </div>
        </div>
      </div>
    </div>

    <div class="hud p-sonar">
      <div class="hd"><span class="ttl">SONAR HC-SR04</span><span>SWEEP <b id="radarAngleText2" class="orb">90&deg;</b></span></div>
      <div class="radar-wrap">
        <svg class="radar-svg" viewBox="0 0 200 110">
          <path class="radar-ring" d="M 40 100 A 60 60 0 0 1 160 100"></path>
          <path class="radar-ring outer" d="M 10 100 A 90 90 0 0 1 190 100"></path>
          <path class="radar-ring" d="M 70 100 A 30 30 0 0 1 130 100"></path>
          <line class="radar-cross" x1="10" y1="100" x2="190" y2="100"></line>
          <line class="radar-cross" x1="100" y1="100" x2="100" y2="10"></line>
          <text class="radar-tick" x="6" y="108" text-anchor="middle">0&deg;</text>
          <text class="radar-tick" x="100" y="8" text-anchor="middle">90&deg;</text>
          <text class="radar-tick" x="194" y="108" text-anchor="middle">180&deg;</text>
          <text class="radar-range-lbl" x="103" y="72" text-anchor="start">60</text>
          <text class="radar-range-lbl" x="103" y="42" text-anchor="start">90</text>
          <text class="radar-range-lbl" x="103" y="14" text-anchor="start">120cm</text>
          <g class="radar-sweep-group" id="radarSweepGroup">
            <polygon class="radar-sweep-fan" points="100,100 81.3,12 118.7,12"></polygon>
            <line class="radar-sweep" x1="100" y1="100" x2="100" y2="14"></line>
            <g class="radar-blip-group" id="radarBlipGroup" style="opacity:0">
              <circle class="radar-blip-ring" id="radarBlipRing" cx="100" cy="100" r="6"></circle>
              <circle class="radar-blip" id="radarBlip" cx="100" cy="100" r="3.2"></circle>
            </g>
          </g>
        </svg>
      </div>
      <div class="radar-readout"><b id="sonarValueText">--</b><span>CM</span></div>
    </div>
  </section>

  <!-- ===================== RIGHT COLUMN: control override joystick ===================== -->
  <section class="hud col p-ctrl">
    <div class="ctrl-hd">
      <div class="l"><span class="ttl">CONTROL OVERRIDE</span><span class="tag">DRIVE STICK</span></div>
      <div class="r">
        <span>DIR:<b id="driveStickHeading">IDLE</b></span>
        <span>MAG:<b class="cy" id="driveStickMag">0%</b></span>
      </div>
    </div>

    <div class="stick-area">
      <div class="vec-row">
        <span>X:<b class="cy" id="driveCoordX">0.00</b></span>
        <span class="dot-sep">&bull;</span>
        <span>Y:<b class="cy" id="driveCoordY">0.00</b></span>
        <span class="dot-sep">&bull;</span>
        <span>RAD:<b class="yl" id="driveRad">0&deg;</b></span>
      </div>

      <div class="joy-base" id="driveJoystickBase">
        <div class="ring ring1"></div>
        <div class="ring ring2"></div>
        <div class="ring ring3"></div>
        <div class="axis-h"></div>
        <div class="axis-v"></div>
        <div class="diag1"></div>
        <div class="diag2"></div>
        <span class="dirlbl dl-up"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M5 15l7-7 7 7"></path></svg>FWD</span>
        <span class="dirlbl dl-down">REV<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M19 9l-7 7-7-7"></path></svg></span>
        <span class="dirlbl dl-left"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M15 19l-7-7 7-7"></path></svg>LEFT</span>
        <span class="dirlbl dl-right">RGHT<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"><path d="M9 5l7 7-7 7"></path></svg></span>

        <div class="thumb" id="driveJoystickThumb">
          <div class="in1"><div class="in2"><div class="in3"></div></div></div>
          <div class="notch n-top"></div>
          <div class="notch n-bottom"></div>
          <div class="notch n-left"></div>
          <div class="notch n-right"></div>
        </div>
      </div>

      <div class="state-row"><span class="lbl">STATE:</span><span class="state-badge" id="driveActiveBadge">IDLE</span></div>
    </div>

    <div class="throttle-row">
      <label>THROTTLE</label>
      <input type="range" id="throttleRange" min="80" max="255" value="200">
      <span id="throttleValue">200</span>
    </div>
  </section>
</main>

<script>
const $ = id => document.getElementById(id);
const NAMA = {F:'FORWARD', B:'REVERSE', L:'TURN LEFT', R:'TURN RIGHT', S:'STOPPED'};
const t0 = Date.now();

/* ---------------- terminal log ---------------- */
let paused = false;
function log(cls, msg){
  if(paused && cls === 'us') return;
  const s = Math.floor((Date.now() - t0) / 1000);
  const ts = String(Math.floor(s/60)).padStart(2,'0') + ':' + String(s%60).padStart(2,'0');
  const d = document.createElement('div');
  d.innerHTML = '<span class="t">[' + ts + ']</span> ' + (cls ? '<span class="' + cls + '">' + msg + '</span>' : msg);
  const term = $('term');
  term.appendChild(d);
  while(term.children.length > 80) term.removeChild(term.firstChild);
  term.scrollTop = term.scrollHeight;
}
$('btnTerminalClear').onclick = () => $('term').innerHTML = '';
$('btnTerminalPause').onclick = e => { paused = !paused; e.target.textContent = paused ? 'RESUME' : 'PAUSE'; };

/* ---------------- request helper ---------------- */
async function api(path){
  try{
    const r = await fetch(path, {cache:'no-store'});
    return r.ok ? await r.text() : null;
  }catch(e){ return null; }
}

/* ---------------- online badge ---------------- */
function setOnline(on){
  $('conn').classList.toggle('on', on);
  $('connText').textContent = on ? 'ONLINE' : 'OFFLINE';
}

/* ---------------- latency sparkline ---------------- */
const LAT_LEN = 20;
let latHistory = [];
function drawLatency(ms){
  latHistory.push(ms);
  if(latHistory.length > LAT_LEN) latHistory.shift();
  const w = 40, h = 16;
  const scaleMax = Math.max(120, ...latHistory);
  const step = w / Math.max(1, LAT_LEN - 1);
  const offset = LAT_LEN - latHistory.length;
  const pts = latHistory.map((v, i) => {
    const x = (offset + i) * step;
    const y = h - Math.min(h, (v / scaleMax) * h);
    return x.toFixed(1) + ',' + y.toFixed(1);
  }).join(' ');
  const col = ms < 80 ? 'var(--green)' : ms < 250 ? 'var(--yellow)' : 'var(--red)';
  const poly = $('latPoly');
  poly.setAttribute('points', pts);
  poly.setAttribute('stroke', col);
  const lb = $('wf-lat');
  lb.style.color = col;
  lb.innerHTML = Math.round(ms) + '<span class="unit">ms</span>';
}

/* ---------------- servo ---------------- */
let currentServoAngle = 90;
const presetBtns = document.querySelectorAll('#presets button');
function updateServoAngle(a){
  currentServoAngle = Math.max(0, Math.min(180, a));
  const cssDeg = currentServoAngle - 90;
  $('servoNeedleGroup').style.transform = 'rotate(' + cssDeg + 'deg)';
  $('radarSweepGroup').style.transform = 'rotate(' + cssDeg + 'deg)';
  $('radarAngleText2').innerHTML = currentServoAngle + '&deg;';
  $('servoHeaderAngle').textContent = currentServoAngle;
  presetBtns.forEach(b => b.classList.toggle('on', +b.dataset.angle === currentServoAngle));
}
async function setServo(a){
  a = Math.max(0, Math.min(180, a));
  const r = await api('/servo?angle=' + a);
  if(r !== null){ updateServoAngle(parseInt(r)); log('sv', 'SERVO SET: ' + parseInt(r) + '&deg;'); }
}
presetBtns.forEach(b => b.onclick = () => setServo(+b.dataset.angle));
updateServoAngle(90);

/* ---------------- radar blip (maps sonar distance onto the sweep line) ---------------- */
const RADAR_MAX_CM = 120;
function setRadarBlip(distCm){
  const grp = $('radarBlipGroup'), dot = $('radarBlip'), ring = $('radarBlipRing');
  if(distCm == null || distCm < 0 || distCm > RADAR_MAX_CM){
    grp.style.opacity = 0;
    return;
  }
  grp.style.opacity = 1;
  const cy = 100 - (14 + Math.min(1, distCm / RADAR_MAX_CM) * 76);
  dot.setAttribute('cy', cy.toFixed(1));
  ring.setAttribute('cy', cy.toFixed(1));
}

/* ---------------- throttle ---------------- */
let baseSpeed = 200;
let spT;
$('throttleRange').oninput = e => {
  baseSpeed = +e.target.value;
  $('throttleValue').textContent = baseSpeed;
  clearTimeout(spT);
  spT = setTimeout(() => { api('/speed?v=' + baseSpeed); log('cm', 'THR = ' + baseSpeed); }, 120);
};

/* ---------------- drive joystick -> real /move + /speed ---------------- */
(function initJoystick(){
  const base = $('driveJoystickBase');
  const thumb = $('driveJoystickThumb');

  let dragging = false, activePointerId = null, curDir = 'S', hold = null;
  let lastSpeedSent = 0, lastSpeedTime = 0;

  function sendDir(d){ api('/move?dir=' + d); }

  function getMaxRadius(){
    const rect = base.getBoundingClientRect();
    const kRect = thumb.getBoundingClientRect();
    return (rect.width / 2) - (kRect.width / 2) - 2;
  }

  function applyState(heading, dir, powerPercent){
    const idle = (dir === 'S');
    $('driveStickHeading').textContent = idle ? 'IDLE' : heading;
    $('driveStickMag').textContent = powerPercent + '%';
    $('driveActiveBadge').textContent = idle ? 'IDLE' : heading;
    $('driveActiveBadge').classList.toggle('act', !idle);
  }

  function update(clientX, clientY){
    const rect = base.getBoundingClientRect();
    const cx = rect.left + rect.width/2, cy = rect.top + rect.height/2;
    let dx = clientX - cx, dy = clientY - cy;
    const maxR = getMaxRadius();
    const dist = Math.hypot(dx, dy);
    if(dist > maxR){
      const ang = Math.atan2(dy, dx);
      dx = Math.cos(ang) * maxR; dy = Math.sin(ang) * maxR;
    }
    thumb.style.transition = 'none';
    thumb.style.transform = 'translate(' + dx.toFixed(1) + 'px,' + dy.toFixed(1) + 'px)';

    const normX = dx / maxR, normY = -(dy / maxR);
    const clampedDist = Math.min(dist, maxR);
    const powerPercent = Math.round((clampedDist / maxR) * 100);
    let deg = Math.round((Math.atan2(dx, -dy) * 180) / Math.PI);
    if(deg < 0) deg += 360;

    $('driveCoordX').textContent = (normX >= 0 ? '+' : '') + normX.toFixed(2);
    $('driveCoordY').textContent = (normY >= 0 ? '+' : '') + normY.toFixed(2);
    $('driveRad').innerHTML = deg + '&deg;';

    let heading = 'STOPPED', dir = 'S';
    if(powerPercent < 12){
      heading = 'IDLE'; dir = 'S';
    } else if(deg >= 315 || deg < 45){ heading = 'FORWARD'; dir = 'F'; }
    else if(deg >= 45 && deg < 135){ heading = 'TURN RIGHT'; dir = 'R'; }
    else if(deg >= 135 && deg < 225){ heading = 'REVERSE'; dir = 'B'; }
    else { heading = 'TURN LEFT'; dir = 'L'; }

    applyState(heading, dir, powerPercent);

    if(dir !== curDir){
      curDir = dir;
      sendDir(dir);
      log('cm', 'CMD &gt; ' + (dir === 'S' ? 'STOPPED' : NAMA[dir]));
    }

    if(dir !== 'S'){
      const now = Date.now();
      if(baseSpeed !== lastSpeedSent && now - lastSpeedTime > 150){
        lastSpeedSent = baseSpeed; lastSpeedTime = now;
        api('/speed?v=' + baseSpeed);
      }
    }
  }

  function onDown(e){
    e.preventDefault();
    dragging = true; activePointerId = e.pointerId;
    base.setPointerCapture(e.pointerId);
    base.classList.add('active'); thumb.classList.add('active');
    update(e.clientX, e.clientY);
    if(hold) clearInterval(hold);
    hold = setInterval(() => { if(curDir !== 'S') sendDir(curDir); }, 250);
    log('cm', 'DRIVE OVERRIDE ACTIVE');
  }
  function onMove(e){
    if(!dragging || e.pointerId !== activePointerId) return;
    e.preventDefault();
    update(e.clientX, e.clientY);
  }
  function onUp(e){
    if(!dragging || e.pointerId !== activePointerId) return;
    e.preventDefault();
    dragging = false;
    try{ base.releasePointerCapture(e.pointerId); }catch(err){}
    activePointerId = null;
    if(hold){ clearInterval(hold); hold = null; }

    base.classList.remove('active'); thumb.classList.remove('active');
    thumb.style.transition = 'transform .2s cubic-bezier(.175,.885,.32,1.275)';
    thumb.style.transform = 'translate(0px,0px)';

    $('driveCoordX').textContent = '0.00';
    $('driveCoordY').textContent = '0.00';
    $('driveStickMag').textContent = '0%';
    $('driveRad').innerHTML = '0&deg;';
    applyState('IDLE', 'S', 0);

    if(curDir !== 'S'){ curDir = 'S'; sendDir('S'); log('cm', 'DRIVE OVERRIDE NEUTRAL'); }
  }

  base.addEventListener('pointerdown', onDown);
  base.addEventListener('pointermove', onMove);
  base.addEventListener('pointerup', onUp);
  base.addEventListener('pointercancel', onUp);
})();

/* ---------------- mode autonomous ---------------- */
const AUTO_ST = ['MAJU','STOPPING','MUNDUR','SCAN KANAN','SCAN KIRI','BELOK'];
let autoOn = false, lastAutoSt = -1;
function syncMode(a, st){
  const on = !!a;
  if(on !== autoOn){
    autoOn = on; lastAutoSt = -1;
    document.body.classList.toggle('auto', on);
    $('modeBtn').classList.toggle('on', on);
    log('cm', on ? 'MODE &gt; AUTONOMOUS' : 'MODE &gt; MANUAL');
    if(!on){ $('driveActiveBadge').textContent = 'IDLE'; $('driveActiveBadge').classList.remove('act'); }
  }
  if(on && st !== lastAutoSt){
    lastAutoSt = st;
    const nm = AUTO_ST[st] || '?';
    $('driveActiveBadge').textContent = nm;
    $('driveActiveBadge').classList.add('act');
    log('cm', 'AUTO &gt; ' + nm);
  }
}
$('modeBtn').onclick = async () => {
  const r = await api('/mode?m=' + (autoOn ? 0 : 1));
  if(r !== null) syncMode(parseInt(r), 0);
};

/* ---------------- status polling ---------------- */
let busy = false, lastSrv = -1;
async function poll(){
  if(busy) return; busy = true;
  const tReq = performance.now();
  const r = await api('/status');
  const rtt = performance.now() - tReq;
  busy = false;
  if(r === null){ setOnline(false); drawLatency(999); return; }
  try{
    const j = JSON.parse(r);
    setOnline(true);
    drawLatency(rtt);

    if(j.s !== lastSrv){ updateServoAngle(j.s); lastSrv = j.s; }
    if(j.p != null && +$('throttleRange').value !== j.p && document.activeElement !== $('throttleRange')){
      $('throttleRange').value = j.p; $('throttleValue').textContent = j.p; baseSpeed = j.p;
    }
    if(j.ch != null) $('wf-ch').textContent = '0' + j.ch;
    if(j.a != null) syncMode(j.a, j.st);

    const de = $('sonarValueText');
    if(j.d < 0){
      de.textContent = 'ERR';
      setRadarBlip(-1);
      log('us', 'SNR: LOSS');
    }else{
      de.textContent = j.d.toFixed(1);
      setRadarBlip(j.d);
      log('us', 'SNR: ' + j.d.toFixed(1) + ' cm');
    }
  }catch(e){ log('er', 'SYS.ERR: Data parse'); }
}
setInterval(poll, 300);
poll();
log('cm', 'INIT AVOIDER_OS...');
</script>
</body>
</html>
)rawliteral";

void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
#else
  ledcSetup(CH_A, 1000, 8);
  ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, 1000, 8);
  ledcAttachPin(PWMB, CH_B);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setMotor(int in1, int in2, int pwmPin, int ch, int dir, bool invert) {
  if (invert) dir = -dir;
  if (dir > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, ch, speedVal);
  } else if (dir < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwmPin, ch, speedVal);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, ch, 0);
  }
}

void drive(char d) {
  motion = d;
  int left = 0, right = 0;
  switch (d) {
    case 'F': left =  1; right =  1; break;   
    case 'B': left = -1; right = -1; break;   
    case 'L': left = -1; right =  1; break;   
    case 'R': left =  1; right = -1; break;   
    default : motion = 'S';                   
  }
  setMotor(AIN1, AIN2, PWMA, CH_A, left,  INVERT_LEFT);
  setMotor(BIN1, BIN2, PWMB, CH_B, right, INVERT_RIGHT);
  Serial.printf("[MOVE] %c | speed %d\n", motion, speedVal);
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);   
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

const float OBSTACLE_DIST = 30.0;
const int   FORWARD_SPEED = 150;
const int   REVERSE_SPEED = 120;
const int   TURN_SPEED    = 100;
const unsigned long REVERSE_TIME = 400;
const unsigned long TURN_TIME    = 350;
const unsigned long SERVO_SETTLE = 300;
const int FRONT_ANGLE = 90;
const int RIGHT_ANGLE = 30;
const int LEFT_ANGLE  = 150;

enum AutoState { MAJU, STOPPING, MUNDUR, SCAN_KANAN, SCAN_KIRI, BELOK };
AutoState autoState = MAJU;

float currentDistance = 0.0;
float rightDistance   = 0.0;
float leftDistance    = 0.0;
int   turnDirection   = 0;
unsigned long stateTimer = 0;

void autoSetMotor(int in1, int in2, int pwmPin, int ch, int dir, int spd, bool invert) {
  if (invert) dir = -dir;
  spd = constrain(spd, 0, 255);
  if (dir > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, ch, spd);
  } else if (dir < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwmPin, ch, spd);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, ch, 0);
  }
}

void driveForward(int spd) {
  autoSetMotor(AIN1, AIN2, PWMA, CH_A,  1, spd, INVERT_LEFT);
  autoSetMotor(BIN1, BIN2, PWMB, CH_B,  1, spd, INVERT_RIGHT);
}

void driveReverse(int spd) {
  autoSetMotor(AIN1, AIN2, PWMA, CH_A, -1, spd, INVERT_LEFT);
  autoSetMotor(BIN1, BIN2, PWMB, CH_B, -1, spd, INVERT_RIGHT);
}

void driveStop() {
  autoSetMotor(AIN1, AIN2, PWMA, CH_A, 0, 0, INVERT_LEFT);
  autoSetMotor(BIN1, BIN2, PWMB, CH_B, 0, 0, INVERT_RIGHT);
}

void driveTurn(int direction, int spd) {
  if (direction > 0) {
    autoSetMotor(AIN1, AIN2, PWMA, CH_A, -1, spd, INVERT_LEFT);
    autoSetMotor(BIN1, BIN2, PWMB, CH_B,  1, spd, INVERT_RIGHT);
  } else {
    autoSetMotor(AIN1, AIN2, PWMA, CH_A,  1, spd, INVERT_LEFT);
    autoSetMotor(BIN1, BIN2, PWMB, CH_B, -1, spd, INVERT_RIGHT);
  }
}

float readDistanceAuto() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000UL);
  if (duration == 0) return 400.0;
  return (duration * 0.0343) / 2.0;
}

void setRadar(int angle) {
  angle = constrain(angle, 0, 180);
  servoAngle = angle;
  servo.write(angle);
}

void publishDistance() {
  distanceCm = (currentDistance >= 400.0) ? -1 : currentDistance;
}

void autoStep() {
  if (autoState == MAJU) {
    setRadar(FRONT_ANGLE);
    currentDistance = readDistanceAuto();
    publishDistance();
    if (currentDistance <= OBSTACLE_DIST) {
      driveStop();
      autoState = STOPPING;
      stateTimer = millis();
    } else {
      driveForward(FORWARD_SPEED);
    }
  }
  else if (autoState == STOPPING) {
    driveStop();
    if (millis() - stateTimer >= 150) {
      autoState = MUNDUR;
      stateTimer = millis();
    }
  }
  else if (autoState == MUNDUR) {
    setRadar(FRONT_ANGLE);
    driveReverse(REVERSE_SPEED);
    currentDistance = readDistanceAuto();
    publishDistance();
    if (millis() - stateTimer >= REVERSE_TIME) {
      driveStop();
      setRadar(RIGHT_ANGLE);
      autoState = SCAN_KANAN;
      stateTimer = millis();
    }
  }
  else if (autoState == SCAN_KANAN) {
    driveStop();
    if (millis() - stateTimer >= SERVO_SETTLE) {
      rightDistance = readDistanceAuto();
      currentDistance = rightDistance;
      publishDistance();
      setRadar(LEFT_ANGLE);
      autoState = SCAN_KIRI;
      stateTimer = millis();
    }
  }
  else if (autoState == SCAN_KIRI) {
    driveStop();
    if (millis() - stateTimer >= SERVO_SETTLE) {
      leftDistance = readDistanceAuto();
      currentDistance = leftDistance;
      publishDistance();
      if (leftDistance > rightDistance) turnDirection = 1;
      else                              turnDirection = -1;
      setRadar(FRONT_ANGLE);
      autoState = BELOK;
      stateTimer = millis();
    }
  }
  else if (autoState == BELOK) {
    driveTurn(turnDirection, TURN_SPEED);
    if (millis() - stateTimer >= TURN_TIME) {
      driveStop();
      setRadar(FRONT_ANGLE);
      autoState = MAJU;
      stateTimer = millis();
    }
  }
}

void setAutoMode(bool on) {
  if (on == autoMode) return;
  drive('S');                       
  autoMode = on;
  if (on) {
    setRadar(FRONT_ANGLE);
    autoState  = MAJU;
    stateTimer = millis();
    Serial.println("[MODE] AUTONOMOUS");
  } else {
    driveStop();
    setRadar(90);
    Serial.println("[MODE] MANUAL");
  }
}

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleMove() {
  if (autoMode) { server.send(200, "text/plain", "AUTO"); return; }  
  if (server.hasArg("dir")) {
    char d = server.arg("dir")[0];
    if (d == 'F' || d == 'B' || d == 'L' || d == 'R' || d == 'S') {
      lastCmd = millis();
      if (d != motion) drive(d);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("v")) {
    speedVal = constrain(server.arg("v").toInt(), 80, 255);
    if (motion != 'S') drive(motion);   
  }
  server.send(200, "text/plain", String(speedVal));
}

void handleServo() {
  if (autoMode) { server.send(200, "text/plain", String(servoAngle)); return; }   
  if (server.hasArg("angle")) {
    int a = server.arg("angle").toInt();
    a = constrain(a, 0, 180);
    a = ((a + 15) / 30) * 30;           
    a = constrain(a, 0, 180);
    servoAngle = a;
    servo.write(servoAngle);
    Serial.printf("[SERVO] %d degrees\n", servoAngle);
  }
  server.send(200, "text/plain", String(servoAngle));
}

void handleMode() {
  if (server.hasArg("m")) setAutoMode(server.arg("m").toInt() == 1);
  server.send(200, "text/plain", autoMode ? "1" : "0");
}

void handleStatus() {
  char buf[160];
  snprintf(buf, sizeof(buf), "{\"d\":%.1f,\"m\":\"%c\",\"s\":%d,\"p\":%d,\"ch\":%d,\"sta\":%d,\"a\":%d,\"st\":%d}",
           distanceCm, motion, servoAngle, speedVal, WiFi.channel(), WiFi.softAPgetStationNum(),
           autoMode ? 1 : 0, (int)autoState);
  server.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(STBY, HIGH);             

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(servoAngle);

  pwmSetup();
  drive('S');

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGW, apMask);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/",       handleRoot);
  server.on("/move",   handleMove);
  server.on("/speed",  handleSpeed);
  server.on("/servo",  handleServo);
  server.on("/mode",   handleMode);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println();
  Serial.println("=== ROBOT AVOIDER READY ===");
  Serial.print("WiFi  : "); Serial.println(AP_SSID);
  Serial.print("Pass  : "); Serial.println(AP_PASS);
  Serial.print("Open  : http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  if (autoMode) {
    autoStep();
    delay(15);
    return;
  }

  if (motion != 'S' && millis() - lastCmd > 700) {
    drive('S');
  }

  if (millis() - lastSensor >= 200) {
    lastSensor = millis();
    distanceCm = readDistance();
    if (distanceCm < 0) Serial.println("[ULTRASONIC] no echo");
    else Serial.printf("[ULTRASONIC] %.1f cm\n", distanceCm);
  }
}
