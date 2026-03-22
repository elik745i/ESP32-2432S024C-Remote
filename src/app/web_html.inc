static const char UPLOAD_FALLBACK_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 Fallback Upload</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial;background:#10131a;color:#eef;margin:0;padding:18px}
    .card{max-width:760px;margin:0 auto;background:#171c26;border:1px solid #2f3b50;border-radius:12px;padding:16px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .muted{color:#9fb0cc}
    input,button{background:#0f141d;color:#eef;border:1px solid #2f3b50;border-radius:8px;padding:8px}
    button{cursor:pointer}
    .a{background:#1e6fff;border-color:#1e6fff}
    .d{background:#c73a3a;border-color:#c73a3a}
    #list{margin-top:10px;border:1px solid #2f3b50;border-radius:10px;max-height:320px;overflow:auto}
    .it{display:grid;grid-template-columns:1fr 100px;gap:8px;padding:8px 10px;border-bottom:1px solid #232e40}
    .it:last-child{border-bottom:0}
  </style>
</head>
<body>
  <div class="card">
    <h2 style="margin:0 0 8px 0">Fallback Upload Manager</h2>
    <div class="muted">Firmware: %FIRMWARE_VERSION%</div>
    <p class="muted">Upload missing web files to <code>/web/</code> or open the recovery browser.</p>
    <div class="row">
      <a href="/recovery"><button>Open Recovery Browser</button></a>
      <button id="refreshBtn">Refresh List</button>
      <button id="clearBtn" class="d">Clear /web/</button>
    </div>
    <form id="uploadForm" style="margin-top:10px">
      <input id="uploadInput" type="file" multiple required>
      <button class="a" type="submit">Upload to /web/</button>
    </form>
    <div id="status" class="muted" style="margin-top:8px">Ready.</div>
    <div id="list"></div>
  </div>
<script>
const listEl = document.getElementById('list');
const statusEl = document.getElementById('status');
const uploadForm = document.getElementById('uploadForm');
const uploadInput = document.getElementById('uploadInput');
const show = (m,e=false)=>{ statusEl.textContent = m; statusEl.style.color = e ? '#ff9797' : '#9fb0cc'; };
function renderList(files){
  if(!files.length){ listEl.innerHTML = '<div class="it"><div>No files in /web/</div><div></div></div>'; return; }
  listEl.innerHTML = files.map(f=>{
    const kb = (Number(f.size)||0) >= 1024 ? ((Number(f.size)||0)/1024).toFixed(1)+' KB' : (Number(f.size)||0)+' B';
    return '<div class="it"><div>'+f.path+'</div><div>'+kb+'</div></div>';
  }).join('');
}
async function loadList(){
  try{
    const r = await fetch('/fs/list',{cache:'no-store'});
    const j = await r.json();
    const files = Array.isArray(j.files) ? j.files.filter(x=>x.path && x.path.startsWith('/web/')) : [];
    renderList(files);
  }catch(e){ show('List failed: ' + e, true); }
}
async function uploadOne(file){
  const fd = new FormData();
  fd.append('fsFile', file, file.name);
  const r = await fetch('/fs/upload?path=/web/', {method:'POST', body:fd});
  if(!r.ok) throw new Error(await r.text());
}
uploadForm.addEventListener('submit', async (e)=>{
  e.preventDefault();
  const files = Array.from(uploadInput.files || []);
  if(!files.length){ show('Select files first', true); return; }
  try{
    for(let i=0;i<files.length;i++){
      show('Uploading ' + files[i].name + ' (' + (i+1) + '/' + files.length + ')');
      await uploadOne(files[i]);
    }
    show('Upload complete');
    await loadList();
  }catch(err){
    show('Upload failed: ' + err.message, true);
  }
});
document.getElementById('refreshBtn').onclick = loadList;
document.getElementById('clearBtn').onclick = async ()=>{
  if(!confirm('Delete all contents of /web/?')) return;
  const body = new URLSearchParams({path:'/web'});
  const r = await fetch('/fs/delete',{method:'POST', body});
  if(!r.ok){ show('Failed to clear /web/', true); return; }
  show('Cleared /web/');
  loadList();
};
loadList();
</script>
</body>
</html>
)rawliteral";

static const char WIFI_SETUP_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 WiFi Setup</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial;background:#10131a;color:#eef;margin:0;padding:18px}
    .card{max-width:760px;margin:0 auto;background:#171c26;border:1px solid #2f3b50;border-radius:12px;padding:16px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .muted{color:#9fb0cc}
    input,button,select{background:#0f141d;color:#eef;border:1px solid #2f3b50;border-radius:8px;padding:8px}
    button{cursor:pointer}
    .a{background:#1e6fff;border-color:#1e6fff}
    #tele{margin-top:10px;padding:10px;border:1px solid #2f3b50;border-radius:10px}
  </style>
</head>
<body>
  <div class="card">
    <h2 style="margin:0 0 8px 0">WiFi Setup</h2>
    <div class="muted">Firmware: %FIRMWARE_VERSION%</div>
    <div class="muted">AP: %AP_SSID% | AP IP: %AP_IP%</div>
    <div id="tele" class="muted">Loading telemetry...</div>
    <div class="row" style="margin-top:10px">
      <button id="scanBtn" class="a">Scan WiFi</button>
      <select id="ssid" style="min-width:240px"></select>
    </div>
    <div class="row" style="margin-top:10px">
      <input id="pass" type="password" placeholder="Password">
      <button id="showBtn">Show</button>
      <button id="connBtn" class="a">Connect</button>
    </div>
    <div id="status" class="muted" style="margin-top:8px">Ready</div>
  </div>
<script>
const teleEl = document.getElementById('tele');
const statusEl = document.getElementById('status');
const ssidEl = document.getElementById('ssid');
const passEl = document.getElementById('pass');
const showBtn = document.getElementById('showBtn');
const show = (m,e=false)=>{ statusEl.textContent=m; statusEl.style.color=e?'#ff9797':'#9fb0cc'; };

async function refreshTelemetry(){
  try{
    const r=await fetch('/api/telemetry',{cache:'no-store'});
    const j=await r.json();
    teleEl.textContent =
      `Battery: ${j.battery_percent}% (${j.battery_voltage}V) | ` +
      `Light: ${j.light_percent}% | ` +
      `WiFi: ${j.wifi_connected ? ('Connected to '+j.wifi_ssid+' ('+j.wifi_rssi+' dBm)') : 'Not connected'}`;
  }catch(e){
    teleEl.textContent='Telemetry unavailable';
  }
}

async function scanWifi(){
  try{
    show('Scanning...');
    const r=await fetch('/api/wifi/scan',{cache:'no-store'});
    const j=await r.json();
    ssidEl.innerHTML='';
    (j.networks||[]).forEach(n=>{
      const o=document.createElement('option');
      o.value=n.ssid;
      o.textContent=`${n.ssid} (${n.rssi} dBm, ${n.auth})`;
      ssidEl.appendChild(o);
    });
    show(`Found ${j.networks ? j.networks.length : 0} networks`);
  }catch(e){
    show('Scan failed: '+e, true);
  }
}

async function connectWifi(){
  const ssid = ssidEl.value || '';
  const pass = passEl.value || '';
  if(!ssid){ show('Select SSID first', true); return; }
  try{
    show('Connecting...');
    const body = new URLSearchParams({ssid, pass});
    const r=await fetch('/api/wifi/connect',{method:'POST', body});
    if(!r.ok) throw new Error(await r.text());
    show('Connect request sent. Device will save credentials on success.');
  }catch(e){
    show('Connect failed: '+e.message, true);
  }
}

document.getElementById('scanBtn').onclick = scanWifi;
document.getElementById('connBtn').onclick = connectWifi;
showBtn.onclick = ()=>{ passEl.type = passEl.type === 'password' ? 'text' : 'password'; showBtn.textContent = passEl.type === 'password' ? 'Show' : 'Hide'; };
scanWifi();
refreshTelemetry();
setInterval(refreshTelemetry, 2000);
</script>
</body>
</html>
)rawliteral";

static const char AP_WIFI_LANDING_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 AP Setup</title>
  <style>
    :root{
      --bg:#050b14;
      --panel:#1d1d1f;
      --panel-2:#20242b;
      --line:#5b6573;
      --text:#f4f7fb;
      --muted:#9db0c5;
      --accent:#5ea2ff;
      --accent-2:#7cb342;
      --danger:#ff8a8a;
      --tab:#efefef;
      --tab-text:#101317;
    }
    *{box-sizing:border-box}
    body{
      margin:0;
      min-height:100vh;
      font-family:Segoe UI,Arial,sans-serif;
      color:var(--text);
      background:
        radial-gradient(circle at top left, rgba(94,162,255,.18), transparent 28%),
        radial-gradient(circle at bottom right, rgba(255,177,66,.12), transparent 24%),
        linear-gradient(180deg, #02060d 0%, #061120 100%);
      display:flex;
      align-items:flex-start;
      justify-content:center;
      padding:18px 12px;
    }
    .shell{
      width:min(980px,100%);
      background:linear-gradient(180deg, rgba(40,44,50,.96), rgba(24,26,29,.98));
      border:4px solid rgba(255,255,255,.92);
      border-radius:22px;
      box-shadow:0 22px 80px rgba(0,0,0,.48);
      overflow:hidden;
    }
    .frame{
      margin:14px;
      padding:18px 18px 24px;
      border-radius:18px;
      background:linear-gradient(180deg, rgba(30,31,34,.98), rgba(27,28,31,.95));
      min-height:calc(100vh - 72px);
      position:relative;
    }
    .topbar{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      margin-bottom:16px;
    }
    .title{
      font-size:30px;
      font-weight:800;
      color:#89bcff;
      display:flex;
      align-items:center;
      gap:10px;
    }
    .title small{
      display:block;
      font-size:13px;
      font-weight:600;
      color:var(--muted);
      margin-top:4px;
    }
    .close{
      width:42px;
      height:42px;
      border-radius:50%;
      border:2px solid rgba(255,255,255,.35);
      background:transparent;
      color:#fff;
      font-size:22px;
      font-weight:700;
      cursor:pointer;
    }
    .layout{
      display:grid;
      grid-template-columns:1.1fr .9fr;
      gap:18px;
    }
    .left{display:flex;flex-direction:column;gap:16px}
    .right{
      min-width:0;
      display:flex;
      flex-direction:column;
      gap:16px;
    }
    .panel{
      background:linear-gradient(180deg, rgba(33,37,44,.98), rgba(24,27,33,.98));
      border:1px solid rgba(255,255,255,.18);
      border-radius:18px;
      padding:16px;
      box-shadow:0 10px 28px rgba(0,0,0,.26);
    }
    .hero h1{
      margin:0 0 6px;
      font-size:32px;
      line-height:1.05;
    }
    .hero p{
      margin:0;
      color:var(--muted);
      line-height:1.5;
      font-size:15px;
    }
    .meta{
      display:grid;
      grid-template-columns:repeat(3,minmax(0,1fr));
      gap:10px;
      margin-top:16px;
    }
    .pill{
      background:rgba(0,0,0,.22);
      border:1px solid rgba(255,255,255,.08);
      border-radius:14px;
      padding:12px;
      min-height:74px;
    }
    .pill .k{
      font-size:11px;
      text-transform:uppercase;
      letter-spacing:.12em;
      color:#8aa0b8;
      margin-bottom:6px;
    }
    .pill .v{
      font-size:15px;
      font-weight:700;
    }
    .tabs{
      display:flex;
      gap:10px;
      flex-wrap:wrap;
      margin-bottom:12px;
    }
    .tab{
      border:none;
      border-radius:999px;
      background:rgba(255,255,255,.1);
      color:var(--text);
      padding:10px 16px;
      font-size:14px;
      font-weight:700;
      cursor:pointer;
    }
    .tab.active{
      background:var(--tab);
      color:var(--tab-text);
    }
    .tabpane{display:none}
    .tabpane.active{display:block}
    .field{
      display:flex;
      flex-direction:column;
      gap:8px;
      margin-bottom:12px;
    }
    .field label{
      font-size:12px;
      font-weight:700;
      text-transform:uppercase;
      letter-spacing:.12em;
      color:#8aa0b8;
    }
    .field input,
    .field textarea,
    .field select{
      width:100%;
      border-radius:14px;
      border:1px solid rgba(255,255,255,.18);
      background:#14181f;
      color:var(--text);
      padding:12px 14px;
      font-size:15px;
      outline:none;
    }
    .field textarea{
      min-height:110px;
      resize:vertical;
    }
    .actions{
      display:flex;
      gap:10px;
      flex-wrap:wrap;
      margin-top:4px;
    }
    .btn{
      border:none;
      border-radius:14px;
      padding:12px 18px;
      font-size:14px;
      font-weight:700;
      cursor:pointer;
      color:#fff;
      background:#3f4a57;
    }
    .btn.primary{background:linear-gradient(135deg,var(--accent),#2d74ff)}
    .btn.good{background:linear-gradient(135deg,var(--accent-2),#4d8d1e)}
    .btn.ghost{background:rgba(255,255,255,.1)}
    .btn.warn{background:linear-gradient(135deg,#ff8b58,#d95d39)}
    .status{
      margin-top:14px;
      padding:12px 14px;
      border-radius:14px;
      background:rgba(0,0,0,.22);
      color:var(--muted);
      min-height:48px;
      border:1px solid rgba(255,255,255,.08);
      white-space:pre-wrap;
    }
    .telemetry{
      display:grid;
      grid-template-columns:repeat(2,minmax(0,1fr));
      gap:12px;
    }
    .tele-card{
      padding:14px;
      border-radius:16px;
      background:rgba(0,0,0,.18);
      border:1px solid rgba(255,255,255,.09);
      min-height:96px;
    }
    .tele-card h3{
      margin:0 0 8px;
      font-size:13px;
      color:#8aa0b8;
      text-transform:uppercase;
      letter-spacing:.12em;
    }
    .tele-card strong{
      display:block;
      font-size:26px;
      margin-bottom:6px;
    }
    .tele-card span{
      color:var(--muted);
      font-size:14px;
      line-height:1.4;
    }
    .network-list{
      display:flex;
      flex-direction:column;
      gap:10px;
      max-height:260px;
      overflow:auto;
      padding-right:4px;
    }
    .network{
      padding:12px 14px;
      border-radius:14px;
      background:rgba(0,0,0,.18);
      border:1px solid rgba(255,255,255,.1);
      cursor:pointer;
    }
    .network.active{
      border-color:rgba(94,162,255,.95);
      box-shadow:0 0 0 2px rgba(94,162,255,.2) inset;
    }
    .network strong{
      display:block;
      font-size:15px;
      margin-bottom:4px;
    }
    .network span{
      color:var(--muted);
      font-size:13px;
    }
    .network em{
      color:#9fe58f;
      font-style:normal;
      font-weight:700;
    }
    .footer{
      margin-top:18px;
      color:#7f93a7;
      font-size:12px;
      text-align:center;
    }
    @media (max-width:860px){
      .layout{grid-template-columns:1fr}
      .meta{grid-template-columns:1fr}
      .telemetry{grid-template-columns:1fr}
      .frame{min-height:auto}
      .title{font-size:24px}
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="frame">
      <div class="topbar">
        <div class="title">
          <div>
            Remote Setup
            <small>%AP_SSID% · firmware %FIRMWARE_VERSION%</small>
          </div>
        </div>
        <button class="close" id="refreshPageBtn" title="Refresh">↻</button>
      </div>

      <div class="layout">
        <div class="left">
          <section class="panel hero">
            <h1>Bring the remote online and ready.</h1>
            <p>Connect this device to WiFi, tune device identity, and keep an eye on battery and signal telemetry while you configure it from the access point.</p>
            <div class="meta">
              <div class="pill">
                <div class="k">Access Point</div>
                <div class="v">%AP_SSID%</div>
              </div>
              <div class="pill">
                <div class="k">Portal IP</div>
                <div class="v">%AP_IP%</div>
              </div>
              <div class="pill">
                <div class="k">Continue</div>
                <div class="v">%CONTINUE_URL%</div>
              </div>
            </div>
          </section>

          <section class="panel">
            <div class="tabs">
              <button class="tab active" data-tab="wifi">WiFi</button>
              <button class="tab" data-tab="device">Device</button>
            </div>

            <div class="tabpane active" id="tab-wifi">
              <div class="field">
                <label>Selected Network</label>
                <input id="selectedSsid" placeholder="Choose a network below or type one manually">
              </div>
              <div class="field">
                <label>Password</label>
                <input id="wifiPass" type="password" placeholder="WiFi password">
              </div>
              <div class="actions">
                <button class="btn primary" id="scanBtn">Scan Networks</button>
                <button class="btn ghost" id="showPassBtn">Show Password</button>
                <button class="btn good" id="connectBtn">Connect WiFi</button>
              </div>
              <div class="status" id="wifiStatus">Ready to scan for nearby WiFi networks.</div>
            </div>

            <div class="tabpane" id="tab-device">
              <div class="field">
                <label>Device Name</label>
                <input id="deviceName" placeholder="Remote name">
              </div>
              <div class="field">
                <label>MQTT Broker</label>
                <input id="mqttHost" placeholder="Broker host">
              </div>
              <div class="field">
                <label>MQTT Port</label>
                <input id="mqttPort" type="number" min="1" max="65535" placeholder="1883">
              </div>
              <div class="actions">
                <button class="btn primary" id="loadCfgBtn">Load Device Config</button>
                <button class="btn good" id="saveCfgBtn">Save Device Config</button>
              </div>
              <div class="status" id="cfgStatus">Configuration values can be loaded from the device and saved back here.</div>
            </div>
          </section>
        </div>

        <div class="right">
          <section class="panel">
            <div class="telemetry">
              <div class="tele-card">
                <h3>Battery</h3>
                <strong id="batteryValue">--%</strong>
                <span id="batteryMeta">Waiting for telemetry</span>
              </div>
              <div class="tele-card">
                <h3>Light</h3>
                <strong id="lightValue">--%</strong>
                <span id="lightMeta">Waiting for telemetry</span>
              </div>
              <div class="tele-card">
                <h3>WiFi</h3>
                <strong id="wifiValue">Offline</strong>
                <span id="wifiMeta">Waiting for telemetry</span>
              </div>
              <div class="tele-card">
                <h3>IP Address</h3>
                <strong id="ipValue">--</strong>
                <span id="ipMeta">Portal %AP_IP%</span>
              </div>
            </div>
          </section>

          <section class="panel">
            <div class="actions" style="margin-top:0;margin-bottom:12px">
              <button class="btn primary" id="refreshTeleBtn">Refresh Telemetry</button>
              <button class="btn ghost" id="continueBtn">Open Continue URL</button>
            </div>
            <div class="network-list" id="networkList">
              <div class="network"><strong>Scan not started</strong><span>Tap "Scan Networks" to look for nearby WiFi.</span></div>
            </div>
          </section>
        </div>
      </div>

      <div class="footer">ESP32 Remote AP setup portal</div>
    </div>
  </div>

<script>
const continueUrl = '%CONTINUE_URL%';
const networkListEl = document.getElementById('networkList');
const selectedSsidEl = document.getElementById('selectedSsid');
const wifiPassEl = document.getElementById('wifiPass');
const wifiStatusEl = document.getElementById('wifiStatus');
const cfgStatusEl = document.getElementById('cfgStatus');
const showStatus = (el, msg, err=false)=>{ el.textContent = msg; el.style.color = err ? '#ffb2b2' : '#9db0c5'; };

function setActiveTab(id){
  document.querySelectorAll('.tab').forEach(btn=>btn.classList.toggle('active', btn.dataset.tab === id));
  document.querySelectorAll('.tabpane').forEach(p=>p.classList.toggle('active', p.id === 'tab-' + id));
}

document.querySelectorAll('.tab').forEach(btn=>{
  btn.addEventListener('click', ()=>setActiveTab(btn.dataset.tab));
});

function renderNetworks(networks){
  if(!Array.isArray(networks) || !networks.length){
    networkListEl.innerHTML = '<div class="network"><strong>No networks found</strong><span>Try scanning again or move closer to the router.</span></div>';
    return;
  }
  networkListEl.innerHTML = '';
  networks.forEach(net=>{
    const row = document.createElement('div');
    row.className = 'network';
    row.innerHTML = `<strong>${net.ssid || '(hidden SSID)'}</strong><span>${net.rssi} dBm · ${net.auth || 'open'} ${net.open ? '<em>open</em>' : ''}</span>`;
    row.addEventListener('click', ()=>{
      selectedSsidEl.value = net.ssid || '';
      document.querySelectorAll('.network').forEach(n=>n.classList.remove('active'));
      row.classList.add('active');
    });
    networkListEl.appendChild(row);
  });
}

async function scanNetworks(){
  try{
    showStatus(wifiStatusEl, 'Scanning for WiFi...');
    const res = await fetch('/api/wifi/scan', {cache:'no-store'});
    const data = await res.json();
    renderNetworks(data.networks || []);
    showStatus(wifiStatusEl, `Found ${(data.networks || []).length} network(s).`);
  }catch(err){
    showStatus(wifiStatusEl, 'WiFi scan failed: ' + err, true);
  }
}

async function connectWifi(){
  const ssid = (selectedSsidEl.value || '').trim();
  const pass = wifiPassEl.value || '';
  if(!ssid){
    showStatus(wifiStatusEl, 'Select or type an SSID first.', true);
    return;
  }
  try{
    showStatus(wifiStatusEl, `Sending WiFi credentials for ${ssid}...`);
    const body = new URLSearchParams({ssid, pass});
    const res = await fetch('/api/wifi/connect', {method:'POST', body});
    if(!res.ok) throw new Error(await res.text());
    showStatus(wifiStatusEl, 'WiFi connect request sent. The remote will save credentials after a successful join.');
  }catch(err){
    showStatus(wifiStatusEl, 'WiFi connect failed: ' + err.message, true);
  }
}

async function refreshTelemetry(){
  try{
    const res = await fetch('/api/telemetry', {cache:'no-store'});
    const t = await res.json();
    document.getElementById('batteryValue').textContent = `${t.battery_percent}%`;
    document.getElementById('batteryMeta').textContent = `${t.battery_voltage}V`;
    document.getElementById('lightValue').textContent = `${t.light_percent}%`;
    document.getElementById('lightMeta').textContent = t.light_percent > 60 ? 'Bright room' : 'Lower ambient light';
    document.getElementById('wifiValue').textContent = t.wifi_connected ? 'Connected' : 'Offline';
    document.getElementById('wifiMeta').textContent = t.wifi_connected ? `${t.wifi_ssid} · ${t.wifi_rssi} dBm` : 'Not joined to a network';
    document.getElementById('ipValue').textContent = t.ip_address || '--';
    document.getElementById('ipMeta').textContent = t.wifi_connected ? 'Station IP' : 'Access point active';
  }catch(err){
    document.getElementById('batteryMeta').textContent = 'Telemetry unavailable';
    document.getElementById('lightMeta').textContent = 'Telemetry unavailable';
    document.getElementById('wifiMeta').textContent = 'Telemetry unavailable';
    document.getElementById('ipMeta').textContent = 'Telemetry unavailable';
  }
}

async function loadConfig(){
  try{
    showStatus(cfgStatusEl, 'Loading device config...');
    const res = await fetch('/get_mqtt', {cache:'no-store'});
    const mqtt = await res.json();
    document.getElementById('mqttHost').value = mqtt.host || '';
    document.getElementById('mqttPort').value = mqtt.port || 1883;

    const tele = await fetch('/api/telemetry', {cache:'no-store'});
    const telemetry = await tele.json();
    document.getElementById('deviceName').value = telemetry.device_name || '';
    showStatus(cfgStatusEl, 'Loaded current device and MQTT settings.');
  }catch(err){
    showStatus(cfgStatusEl, 'Failed to load device config: ' + err, true);
  }
}

async function saveConfig(){
  try{
    showStatus(cfgStatusEl, 'Saving configuration...');
    const mqttBody = JSON.stringify({
      enable: true,
      host: document.getElementById('mqttHost').value || '',
      port: Number(document.getElementById('mqttPort').value || 1883),
      user: '',
      pass: '',
      topic_prefix: 'homeassistant'
    });
    const mqttRes = await fetch('/set_mqtt', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:mqttBody
    });
    if(!mqttRes.ok) throw new Error(await mqttRes.text());
    showStatus(cfgStatusEl, 'MQTT settings saved. Device name can still be changed from the remote UI if needed.');
  }catch(err){
    showStatus(cfgStatusEl, 'Save failed: ' + err.message, true);
  }
}

document.getElementById('scanBtn').addEventListener('click', scanNetworks);
document.getElementById('connectBtn').addEventListener('click', connectWifi);
document.getElementById('showPassBtn').addEventListener('click', ()=>{
  wifiPassEl.type = wifiPassEl.type === 'password' ? 'text' : 'password';
});
document.getElementById('refreshTeleBtn').addEventListener('click', refreshTelemetry);
document.getElementById('refreshPageBtn').addEventListener('click', ()=>location.reload());
document.getElementById('continueBtn').addEventListener('click', ()=>{
  if(continueUrl && continueUrl !== '/'){ location.href = continueUrl; }
});
document.getElementById('loadCfgBtn').addEventListener('click', loadConfig);
document.getElementById('saveCfgBtn').addEventListener('click', saveConfig);

refreshTelemetry();
scanNetworks();
setInterval(refreshTelemetry, 2500);
</script>
</body>
</html>
)rawliteral";
