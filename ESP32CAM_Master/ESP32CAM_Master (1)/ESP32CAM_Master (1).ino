// ================================================================
//  ESP32CAM_Master.ino
//  Vai trò: WiFi Access Point + Camera Stream + Web UI
//           Giao tiếp UART với ESP32 Slave
//
//  Kết nối UART:
//    ESP32-CAM GPIO1 (TX) ──► ESP32 GPIO16 (UART2 RX)
//    ESP32-CAM GPIO3 (RX) ◄── ESP32 GPIO17 (UART2 TX)
//    ESP32-CAM GND        ──── ESP32 GND
//
//  Thư viện cần cài (Library Manager):
//    - AsyncTCP
//    - ESPAsyncWebServer
// ================================================================

#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
const char* AI_SERVER = "http://192.168.4.2:5000/detect"; // IP máy tính

String lastTrafficLight = "NONE";
unsigned long lastAICheck = 0;
const unsigned long AI_INTERVAL = 500; // ms — kiểm tra mỗi 0.5 giây
int lastAICode = 0;

// ----------------------------------------------------------------
//  CAMERA PIN (AI-Thinker ESP32-CAM)
// ----------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_PIN 4  // Flash LED của ESP32-CAM AI-Thinker
// ----------------------------------------------------------------
//  UART GIAO TIẾP VỚI ESP32 SLAVE
//  GPIO1=TX, GPIO3=RX — dùng Serial (UART0)
//  ⚠️ Khi nạp code phải rút dây TX/RX nối với ESP32
// ----------------------------------------------------------------
// Dùng Serial mặc định (UART0) — tốc độ 9600
// Serial.print() → gửi lệnh đến ESP32
// Serial.available() → nhận dữ liệu từ ESP32

// ----------------------------------------------------------------
//  WIFI
// ----------------------------------------------------------------
const char* ssid     = "ESP32-CAM-CAR";
const char* password = "12345678";

// ----------------------------------------------------------------
//  WEB SERVER & WEBSOCKET
// ----------------------------------------------------------------
AsyncWebServer server(80);
AsyncWebSocket  wsControl("/Control");

void checkTrafficLight() {
  if (millis() - lastAICheck < AI_INTERVAL) return;
  lastAICheck = millis();

  Serial.println("AI START");

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    lastAICode = -1001; // camera frame fail
    return;
  }

  HTTPClient http;

  Serial.println("HTTP BEGIN");

  http.begin(AI_SERVER);
  http.addHeader("Content-Type", "image/jpeg");
  http.setConnectTimeout(1500);
  http.setTimeout(4000);

  Serial.println("POSTING...");

  int code = http.POST(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  lastAICode = code;

  Serial.printf("HTTP CODE: %d\n", code);

  {
    char aiBuf[24];
    snprintf(aiBuf, sizeof(aiBuf), "{\"ai\":%d}", lastAICode);
    wsControl.textAll(aiBuf);
  }

  if (code == 200) {
    String body = http.getString();

    Serial.println(body);

    if (body.indexOf("RED") >= 0)
      lastTrafficLight = "RED";
    else if (body.indexOf("GREEN") >= 0)
      lastTrafficLight = "GREEN";
    else
      lastTrafficLight = "NONE";

    Serial.println(lastTrafficLight);

    if (lastTrafficLight == "RED")
      Serial.println("CMD,TrafficLight,1");
    else if (lastTrafficLight == "GREEN")
      Serial.println("CMD,TrafficLight,2");
    else
      Serial.println("CMD,TrafficLight,0");

    char buf[32];
    snprintf(buf, sizeof(buf),
             "{\"tl\":\"%s\"}",
             lastTrafficLight.c_str());

    wsControl.textAll(buf);
  }

  http.end();

  Serial.println("AI END");
}
// ----------------------------------------------------------------
//  TRẠNG THÁI XE (nhận từ ESP32, hiển thị trên web)
// ----------------------------------------------------------------
int  distF   = 999;
int  distL   = 999;
int  distR   = 999;
bool irLeft  = false;
bool irRight = false;
bool autoMode = false;

// ----------------------------------------------------------------
//  HTML GIAO DIỆN WEB
// ----------------------------------------------------------------
const char* htmlPage PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>ESP32-CAM Car</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&family=Share+Tech+Mono&display=swap');

    :root {
      --bg:      #080c14;
      --panel:   #0d1520;
      --border:  #1a3050;
      --accent:  #00c8ff;
      --green:   #00ff88;
      --red:     #ff2244;
      --amber:   #ffaa00;
      --text:    #a0c0d8;
      --dim:     #304860;
    }

    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }

    body {
      background: var(--bg);
      color: var(--text);
      font-family: 'Share Tech Mono', monospace;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 10px 10px 20px;
      overflow-x: hidden;
    }

    body::before {
      content: '';
      position: fixed; inset: 0;
      background:
        linear-gradient(rgba(0,200,255,.025) 1px, transparent 1px),
        linear-gradient(90deg, rgba(0,200,255,.025) 1px, transparent 1px);
      background-size: 32px 32px;
      pointer-events: none; z-index: 0;
    }

    .wrap { position: relative; z-index: 1; width: 100%; max-width: 430px; }

    /* ── HEADER ── */
    header {
      text-align: center;
      padding: 8px 0 12px;
    }
    header h1 {
      font-family: 'Orbitron', sans-serif;
      font-size: 15px; font-weight: 900;
      letter-spacing: 4px;
      color: var(--accent);
      text-shadow: 0 0 24px rgba(0,200,255,.5);
    }
    header p { font-size: 9px; color: var(--dim); letter-spacing: 3px; margin-top: 3px; }

    /* ── LINE STATUS LIGHTS ── */
    .line-panel {
      background: linear-gradient(135deg, rgba(13,21,32,.98), rgba(6,12,20,.98));
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px 14px;
      margin-bottom: 10px;
    }
    .line-head {
      display: flex; justify-content: space-between; align-items: center;
      margin-bottom: 12px;
      font-size: 9px; letter-spacing: 2px;
      color: var(--dim);
    }
    .line-state {
      color: var(--accent);
      font-family: 'Orbitron', sans-serif;
      font-size: 10px; font-weight: 700;
    }
    .lamp-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .lamp-card {
      display: flex; flex-direction: column; align-items: center; gap: 8px;
      background: rgba(4,8,16,.7);
      border: 1px solid rgba(26,48,80,.85);
      border-radius: 8px;
      padding: 12px 8px 10px;
    }
    .lamp {
      width: 44px; height: 44px; border-radius: 50%;
      background: var(--dim);
      box-shadow: inset 0 0 14px rgba(0,0,0,.8);
      transition: all .18s;
    }
    .lamp.red.on {
      background: var(--red);
      box-shadow: 0 0 20px rgba(255,34,68,.65), inset 0 0 10px rgba(255,255,255,.18);
    }
    .lamp.green.on {
      background: var(--green);
      box-shadow: 0 0 20px rgba(0,255,136,.65), inset 0 0 10px rgba(255,255,255,.18);
    }
    .lamp-label {
      font-family: 'Orbitron', sans-serif;
      font-size: 10px; font-weight: 700;
      letter-spacing: 2px;
    }
    .lamp-label.red { color: var(--red); }
    .lamp-label.green { color: var(--green); }

    /* ── MODE BUTTONS ── */
    .mode-row { display: flex; gap: 8px; margin-bottom: 10px; }
    .mode-btn {
      flex: 1; padding: 11px 6px;
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      font-family: 'Orbitron', sans-serif;
      font-size: 10px; font-weight: 700;
      letter-spacing: 2px;
      color: var(--dim);
      cursor: pointer;
      text-align: center;
      transition: all .15s;
      user-select: none; -webkit-user-select: none;
    }
    .mode-btn.on-manual {
      border-color: var(--accent); color: var(--accent);
      background: rgba(0,200,255,.07);
      box-shadow: 0 0 14px rgba(0,200,255,.2);
    }
    .mode-btn.on-auto {
      border-color: var(--green); color: var(--green);
      background: rgba(0,255,136,.07);
      box-shadow: 0 0 14px rgba(0,255,136,.2);
    }

    /* ── STATUS BAR ── */
    .status {
      display: flex; justify-content: center; gap: 34px;
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 8px 10px;
      margin-bottom: 10px;
    }
    .st { display: flex; flex-direction: column; align-items: center; gap: 2px; }
    .st-l { font-size: 8px; color: var(--dim); letter-spacing: 1px; }
    .st-v { font-size: 13px; font-weight: bold; color: var(--accent); }
    .st-v.ok     { color: var(--green); }
    .st-v.warn   { color: var(--amber); }
    .st-v.danger { color: var(--red);   }

    /* ── IR INDICATORS ── */
    .ir-row {
      display: flex; justify-content: center; gap: 12px;
      margin-bottom: 10px;
    }
    .ir-pill {
      display: flex; align-items: center; gap: 6px;
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 5px 14px;
      font-size: 10px; letter-spacing: 1px;
    }
    .ir-led {
      width: 10px; height: 10px; border-radius: 50%;
      background: var(--dim);
      transition: background .1s;
    }
    .ir-led.black { background: var(--red); box-shadow: 0 0 6px var(--red); }

    /* ── DPAD ── */
    .ctrl-box {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 14px 16px 16px;
      margin-bottom: 10px;
      position: relative;
    }
    .ctrl-label {
      text-align: center;
      font-size: 8px; letter-spacing: 3px;
      color: var(--dim); margin-bottom: 12px;
    }
    .dpad {
      display: grid;
      grid-template: repeat(3,54px) / repeat(3,54px);
      gap: 7px;
      margin: 0 auto;
      width: fit-content;
    }
    .db {
      width: 54px; height: 54px;
      background: var(--bg);
      border: 1px solid var(--border);
      border-radius: 9px;
      display: flex; align-items: center; justify-content: center;
      font-size: 24px; color: var(--accent);
      cursor: pointer;
      transition: all .08s;
      position: relative; overflow: hidden;
      user-select: none; -webkit-user-select: none;
    }
    .db::after {
      content: ''; position: absolute; inset: 0;
      background: var(--accent); opacity: 0; transition: opacity .08s;
    }
    .db.act::after, .db:active::after { opacity: .12; }
    .db.act, .db:active {
      border-color: var(--accent);
      box-shadow: 0 0 10px rgba(0,200,255,.3);
      transform: scale(.94);
    }
    .db.stop-btn { font-size: 11px; color: var(--red); border-color: #200a10; }
    .db.stop-btn::after { background: var(--red); }
    .db.stop-btn.act, .db.stop-btn:active { border-color: var(--red); box-shadow: 0 0 10px rgba(255,34,68,.3); }
    .dc { /* empty cell */ }

    /* Auto mode dim overlay */
    .ctrl-box.auto-on::after {
      content: 'AUTO MODE ACTIVE';
      position: absolute; inset: 0;
      background: rgba(0,255,136,.04);
      border-radius: 10px;
      display: flex; align-items: center; justify-content: center;
      font-family: 'Orbitron', sans-serif;
      font-size: 11px; letter-spacing: 3px;
      color: rgba(0,255,136,.4);
      pointer-events: none;
    }
    .ctrl-box.auto-on .db:not(.stop-btn) {
      opacity: .3; pointer-events: none;
    }

    /* ── SLIDERS ── */
    .sliders-box {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px 14px;
    }
    .sl-row { margin-bottom: 12px; }
    .sl-row:last-child { margin-bottom: 0; }
    .sl-head {
      display: flex; justify-content: space-between;
      font-size: 9px; letter-spacing: 2px;
      margin-bottom: 7px;
    }
    .sl-name { color: var(--dim); }
    .sl-val  { color: var(--accent); font-weight: bold; font-size: 11px; }

    input[type=range] {
      -webkit-appearance: none;
      width: 100%; height: 3px;
      background: #0a1828; border-radius: 2px; outline: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 18px; height: 18px; border-radius: 50%;
      background: var(--accent); cursor: pointer;
      box-shadow: 0 0 8px rgba(0,200,255,.5);
    }
    #Light::-webkit-slider-thumb {
      background: #ffc800;
      box-shadow: 0 0 8px rgba(255,200,0,.5);
    }

    /* ── CONN STATUS ── */
    .conn-bar {
      display: flex; justify-content: center; gap: 16px;
      margin-bottom: 8px; font-size: 9px; letter-spacing: 1px;
    }
    .conn-item { display: flex; align-items: center; gap: 4px; color: var(--dim); }
    .conn-dot  { width: 6px; height: 6px; border-radius: 50%; background: var(--dim); }
    .conn-dot.on { background: var(--green); }
  </style>
</head>
<body>
<div class="wrap">

  <header>
    <h1>◼ ROVER CONTROL</h1>
    <p>LINE FOLLOWER · OBSTACLE AVOIDANCE</p>
  </header>

  <!-- Connection status -->
  <div class="conn-bar">
    <div class="conn-item"><div class="conn-dot" id="dotCtrl"></div>CONTROL</div>
    <div class="conn-item"><div class="conn-dot" id="dotNano"></div>ESP32</div>
  </div>

  <!-- Line / traffic light status -->
  <div class="line-panel">
    <div class="line-head">
      <span>LINE DETECT STATUS</span>
      <span class="line-state" id="tlText">NONE</span>
    </div>
    <div class="lamp-row">
      <div class="lamp-card">
        <div class="lamp red" id="lampRed"></div>
        <div class="lamp-label red">RED</div>
      </div>
      <div class="lamp-card">
        <div class="lamp green" id="lampGreen"></div>
        <div class="lamp-label green">GREEN</div>
      </div>
    </div>
  </div>

  <!-- Mode -->
  <div class="mode-row">
    <div class="mode-btn on-manual" id="btnM" onclick="setMode(0)">▶ MANUAL</div>
    <div class="mode-btn"          id="btnA" onclick="setMode(1)">⬆ AUTO LINE</div>
  </div>

  <!-- Status -->
  <div class="status">
    <div class="st">
      <span class="st-l">MODE</span>
      <span class="st-v ok" id="svMode">MANUAL</span>
    </div>
    <div class="st">
      <span class="st-l">AI</span>
      <span class="st-v" id="svAI">---</span>
    </div>
  </div>

  <!-- IR indicators -->
  <div class="ir-row">
    <div class="ir-pill">
      <div class="ir-led" id="irL"></div>
      IR LEFT
    </div>
    <div class="ir-pill">
      IR RIGHT
      <div class="ir-led" id="irR"></div>
    </div>
  </div>

  <!-- D-PAD -->
  <div class="ctrl-box" id="ctrlBox">
    <div class="ctrl-label">DIRECTIONAL CONTROL</div>
    <div class="dpad">
      <div class="dc"></div>
      <div class="db" id="dUp"
           ontouchstart="pd(event);dn('MoveCar','1',this)" ontouchend="pd(event);up('MoveCar','0',this)"
           onmousedown="dn('MoveCar','1',this)" onmouseup="up('MoveCar','0',this)">↑</div>
      <div class="dc"></div>

      <div class="db" id="dLt"
           ontouchstart="pd(event);dn('MoveCar','3',this)" ontouchend="pd(event);up('MoveCar','0',this)"
           onmousedown="dn('MoveCar','3',this)" onmouseup="up('MoveCar','0',this)">←</div>
      <div class="db stop-btn"
           ontouchstart="pd(event);dn('MoveCar','0',this)" ontouchend="pd(event);up('MoveCar','0',this)"
           onmousedown="dn('MoveCar','0',this)" onmouseup="up('MoveCar','0',this)">STOP</div>
      <div class="db" id="dRt"
           ontouchstart="pd(event);dn('MoveCar','4',this)" ontouchend="pd(event);up('MoveCar','0',this)"
           onmousedown="dn('MoveCar','4',this)" onmouseup="up('MoveCar','0',this)">→</div>

      <div class="dc"></div>
      <div class="db" id="dDn"
           ontouchstart="pd(event);dn('MoveCar','2',this)" ontouchend="pd(event);up('MoveCar','0',this)"
           onmousedown="dn('MoveCar','2',this)" onmouseup="up('MoveCar','0',this)">↓</div>
      <div class="dc"></div>
    </div>
  </div>

  <!-- Sliders -->
  <div class="sliders-box">
    <div class="sl-row">
      <div class="sl-head">
        <span class="sl-name">▶ SPEED</span>
        <span class="sl-val" id="spVal">150</span>
      </div>
      <input type="range" min="0" max="255" value="150" id="Speed"
             oninput="spVal.textContent=this.value; send('Speed',this.value)">
    </div>
    <div class="sl-row">
      <div class="sl-head">
        <span class="sl-name">☀ LIGHT</span>
        <span class="sl-val" id="ltVal">0</span>
      </div>
      <input type="range" min="0" max="255" value="0" id="Light"
             oninput="ltVal.textContent=this.value; send('Light',this.value)">
    </div>
  </div>

</div><!-- .wrap -->

<script>
// ── WebSocket ──────────────────────────────────────────────
var wsCtrl;
var isAuto = false;

function initCtrlWS() {
  wsCtrl = new WebSocket("ws://" + location.hostname + "/Control");
  wsCtrl.onopen = () => {
    dotOn('dotCtrl');
    send('Speed', document.getElementById('Speed').value);
    send('Light', document.getElementById('Light').value);
  };
  wsCtrl.onclose = () => {
    dotOff('dotCtrl');
    setTimeout(initCtrlWS, 2000);
  };
  wsCtrl.onmessage = e => {
    try {
      var d = JSON.parse(e.data);
      // IR sensor
      if (d.iL !== undefined) setIR('irL', d.iL);
      if (d.iR !== undefined) setIR('irR', d.iR);
      // Nano kết nối
      if (d.nano !== undefined) {
        d.nano ? dotOn('dotNano') : dotOff('dotNano');
      }
      if (d.ai !== undefined) setAI(d.ai);
      if (d.tl !== undefined) setTrafficLight(d.tl);
    } catch(e) {}
  };
}

function send(k, v) {
  if (wsCtrl && wsCtrl.readyState === 1) wsCtrl.send(k + "," + v);
}

// ── Controls ──────────────────────────────────────────────
function pd(e) { e.preventDefault(); }
function dn(k, v, el) { if(isAuto) return; el.classList.add('act'); send(k, v); }
function up(k, v, el) { el.classList.remove('act'); send(k, v); }

function setMode(m) {
  isAuto = (m === 1);
  document.getElementById('btnM').className = 'mode-btn' + (isAuto ? '' : ' on-manual');
  document.getElementById('btnA').className = 'mode-btn' + (isAuto ? ' on-auto' : '');
  document.getElementById('ctrlBox').className = 'ctrl-box' + (isAuto ? ' auto-on' : '');
  document.getElementById('svMode').textContent = isAuto ? 'AUTO' : 'MANUAL';
  document.getElementById('svMode').className = 'st-v ' + (isAuto ? 'ok' : '');
  send('AutoMode', isAuto ? '1' : '0');
  if (!isAuto) send('MoveCar', '0');
}

function setIR(id, val) {
  var el = document.getElementById(id);
  el.className = 'ir-led' + (val ? ' black' : '');
}

function setTrafficLight(state) {
  var red = document.getElementById('lampRed');
  var green = document.getElementById('lampGreen');
  var text = document.getElementById('tlText');
  red.classList.toggle('on', state === 'RED');
  green.classList.toggle('on', state === 'GREEN');
  text.textContent = state || 'NONE';
  text.style.color = state === 'RED' ? 'var(--red)' :
                     state === 'GREEN' ? 'var(--green)' : 'var(--accent)';
}

function setAI(code) {
  var el = document.getElementById('svAI');
  el.textContent = code;
  if (code === 200) el.className = 'st-v ok';
  else if (code > 0) el.className = 'st-v warn';
  else el.className = 'st-v danger';
}

function dotOn(id, txtId, txt) {
  document.getElementById(id).classList.add('on');
  if (txtId) document.getElementById(txtId).textContent = txt;
}
function dotOff(id, txtId, txt) {
  document.getElementById(id).classList.remove('on');
  if (txtId) document.getElementById(txtId).textContent = txt;
}

window.onload = () => { initCtrlWS(); };
</script>
</body>
</html>
)HTML";

// ================================================================
//  CAMERA SETUP
// ================================================================
bool setupCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk    = XCLK_GPIO_NUM;
  cfg.pin_pclk    = PCLK_GPIO_NUM;
  cfg.pin_vsync   = VSYNC_GPIO_NUM;
  cfg.pin_href    = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn    = PWDN_GPIO_NUM;
  cfg.pin_reset   = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_QVGA;
 cfg.jpeg_quality = 15;                // 15 = đủ tốt cho AI, nhẹ hơn
  cfg.fb_count     = 2; 

  if (esp_camera_init(&cfg) != ESP_OK) {
    return false;
  }
  if (psramFound()) heap_caps_malloc_extmem_enable(20000);
  return true;
}

// ================================================================
//  WEBSOCKET HANDLERS
// ================================================================
void onControlWS(AsyncWebSocket*, AsyncWebSocketClient* client,
                 AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // Gửi trạng thái hiện tại ngay khi kết nối
    char buf[128];
    snprintf(buf, sizeof(buf),
      "{\"dF\":%d,\"dL\":%d,\"dR\":%d,\"iL\":%d,\"iR\":%d,\"nano\":1,\"ai\":%d,\"tl\":\"%s\"}",
      distF, distL, distR, irLeft, irRight, lastAICode, lastTrafficLight.c_str());
    client->text(buf);
    return;
  }
  if (type == WS_EVT_DISCONNECT) {
    // Gửi lệnh dừng xuống ESP32 khi web ngắt kết nối
    Serial.println("CMD,STOP");
    return;
  }
  if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len) return;
    if (info->opcode != WS_TEXT) return;

    String msg = "";
    for (size_t i = 0; i < len; i++) msg += (char)data[i];

    // Parse "Key,Value"
    int comma = msg.indexOf(',');
    if (comma < 0) return;
    String key = msg.substring(0, comma);
    String val = msg.substring(comma + 1);
    if (key == "Light") {
      // Xử lý ngay tại ESP32-CAM, không forward xuống Slave
      int brightness = val.toInt();
      ledcWrite(2, brightness);
      return; // <-- không Serial.println xuống Slave
    }
    // Chuyển tiếp xuống ESP32 qua Serial
    // Format: "CMD,<key>,<value>\n"
    Serial.println("CMD," + key + "," + val);
  }
}

// ================================================================
//  ĐỌC DỮ LIỆU TỪ ESP32
// ================================================================
String esp32Buffer = "";

void readFromESP32() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      // Xử lý dòng hoàn chỉnh
      esp32Buffer.trim();
      if (esp32Buffer.startsWith("TEL,")) {
        // Format: "TEL,dF,dL,dR,irL,irR"
        // Ví dụ:  "TEL,25,80,60,0,1"
        int p = 4;
        auto nextVal = [&]() -> int {
          int comma = esp32Buffer.indexOf(',', p);
          int v;
          if (comma < 0) { v = esp32Buffer.substring(p).toInt(); p = esp32Buffer.length(); }
          else           { v = esp32Buffer.substring(p, comma).toInt(); p = comma + 1; }
          return v;
        };
        distF   = nextVal();
        distL   = nextVal();
        distR   = nextVal();
        irLeft  = nextVal();
        irRight = nextVal();

        // Đẩy telemetry lên tất cả web client
        char buf[128];
        snprintf(buf, sizeof(buf),
          "{\"dF\":%d,\"dL\":%d,\"dR\":%d,\"iL\":%d,\"iR\":%d,\"nano\":1,\"ai\":%d,\"tl\":\"%s\"}",
          distF, distL, distR, (int)irLeft, (int)irRight, lastAICode, lastTrafficLight.c_str());
        wsControl.textAll(buf);
      }
      esp32Buffer = "";
    } else {
      esp32Buffer += c;
    }
  }
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  // UART0 giao tiếp với Nano (GPIO1=TX, GPIO3=RX)
  Serial.begin(9600);

  // WiFi AP
  WiFi.softAP(ssid, password);
  // (Serial.print tắt vì dùng cho Nano — dùng Serial2 nếu cần debug)

  // Web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
    req->send_P(200, "text/html", htmlPage);
  });
  server.onNotFound([](AsyncWebServerRequest* req){
    req->send(404, "text/plain", "Not Found");
  });
  ledcSetup(2, 5000, 8);        // channel 2, 5kHz, 8-bit
  ledcAttachPin(FLASH_PIN, 2);
  ledcWrite(2, 0);
  wsControl.onEvent(onControlWS);
  server.addHandler(&wsControl);

  server.begin();
  if (!setupCamera()) {
    lastAICode = -1002; // camera init fail
  }
}

void loop() {
  readFromESP32();           // Nhận telemetry từ Nano
  wsControl.cleanupClients();
  checkTrafficLight(); 
}
