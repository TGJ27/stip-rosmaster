// STIP dashboard module: views
function home() {
    var _a, _b, _c;
    const ros = state.status.ros || {}, p = state.status.processes || {}, online = !!ros.connected;
    const data = (state.status.events || []).slice(-35).reverse().map(s => `<div class="logline">${esc(s)}</div>`).join('') || '<div class="small muted">No events recorded</div>';
    return `<div class="home-grid"><section class="card home-card">
 <div class="metric"><span>ROS Connection</span><b class="${online ? 'good' : 'bad'}">${online ? 'CONNECTED' : 'OFFLINE'}</b></div>
 <div class="metric"><span>Stage Manager</span><b>${((_a = p.system) === null || _a === void 0 ? void 0 : _a.running) ? 'MANAGED' : ros.modules ? 'CONNECTED' : 'UNAVAILABLE'}</b></div>
 <div class="metric"><span>Navigation</span><b>${((_b = p.navigation) === null || _b === void 0 ? void 0 : _b.running) ? 'ACTIVE' : 'IDLE'}</b></div>
 <div class="metric"><span>Mapping</span><b>${((_c = p.mapping) === null || _c === void 0 ? void 0 : _c.running) ? 'ACTIVE' : 'IDLE'}</b></div>
 <div class="row space" style="margin-top:12px"><h2 style="margin:0">Recent Events</h2></div>
 <div class="event-scroll" id="eventList">${data}</div></section>
 <section class="card home-card"><div class="row space"><h2 style="margin:0">Component Health</h2></div>
 <div class="health-rows" id="componentHealth">${healthTable()}${auxHealth()}</div></section></div>`;
}
function maps() { return `<div class="row" style="justify-content:flex-end;margin-bottom:12px">${btn('+ Create Map', 'createMapDialog()', 'blue')}</div><div class="grid gallery">${state.maps.map(m => `<div class="card tile"><img src="/api/maps/${m.name}.png" alt="Map ${esc(m.name)}" style="cursor:pointer" onclick="previewMap('${m.name}')"><strong>${esc(m.name)}</strong><div class="small muted">${m.missions} mission(s) · ${new Date(m.modified * 1000).toLocaleString()}</div><div class="flexwrap" style="margin-top:12px">${btn('View', `previewMap('${m.name}')`, 'blue s')}${btn('Delete', `deleteMap('${m.name}')`, 'red s')}</div></div>`).join('') || '<div class="empty">No saved maps found. Check the server maps directory if you expected maps here.</div>'}</div>`; }
function missions() { return head('Mission', 'Saved missions and editable waypoint tasks', btn('+ Create Mission', 'createMissionDialog()', 'blue')) + `<div class="grid gallery">${state.missions.map(m => `<div class="card tile" onclick="openMission('${m.id}')"><img src="/api/maps/${m.map}.png"><strong>${esc(m.name)}</strong><div class="small muted">Map: ${esc(m.map)} · ${m.waypoints.length} waypoint(s)</div><div style="margin-top:10px" class="flexwrap">${btn('Open / Edit', `openMission('${m.id}')`, 'blue s')}${btn('Delete', `deleteMission('${m.id}')`, 'red s')}</div></div>`).join('') || '<div class="empty">No missions yet. Create a mission and choose a saved map.</div>'}</div>`; }
function patrolList() { return head('Patrol', 'Select a saved mission to localize and run') + `<div class="grid gallery">${state.missions.filter(m => m.waypoints.length).map(m => `<div class="card tile" onclick="loadPatrol('${m.id}')"><img src="/api/maps/${m.map}.png"><strong>${esc(m.name)}</strong><div class="small muted">${esc(m.map)} · ${m.waypoints.length} waypoint(s)</div>${btn('Load Mission', `loadPatrol('${m.id}')`, 'blue s')}</div>`).join('') || '<div class="empty">No saved missions with waypoints.</div>'}</div>`; }
function settings() { var _a, _b; return head('Settings', 'Operator configuration and integrations') + `<div class="grid" style="grid-template-columns:repeat(auto-fit,minmax(250px,1fr))"><div class="card"><h2>Mapping & Navigation</h2><label class="field">Navigation backend<input value="${esc(state.status.engine || 'stip')}" disabled></label><div class="metric"><span>ROS Bridge</span><span class="${((_a = state.status.ros) === null || _a === void 0 ? void 0 : _a.connected) ? 'good' : 'bad'}">${((_b = state.status.ros) === null || _b === void 0 ? void 0 : _b.connected) ? 'Connected' : 'Offline'}</span></div></div><div class="card"><h2>MQ-6 Clean-air Baseline</h2><div id="gasCalibration">${gasCalibrationPanel()}</div></div><div class="card"><h2>Fire / Smoke AI Confidence</h2><input type="hidden" id="fireSettingsLoaded" value=""><label class="field">Fire confidence (0.10–0.95)<input type="number" id="fireConfInput" min="0.10" max="0.95" step="0.01" value="0.25"></label><label class="field">Smoke confidence (0.10–0.95)<input type="number" id="smokeConfInput" min="0.10" max="0.95" step="0.01" value="0.35"></label><p class="small muted">These are the original C++ detector thresholds. A managed fire-stage restart is required to apply them. Thermal fusion separately verifies actual fire.</p><button class="btn blue" onclick="saveFireSettings()">Save & Restart Detector</button><p class="small muted" id="fireSettingsStatus">Loading actual ROS configuration…</p></div><div class="card"><h2>Fire Tracking Speed</h2><input type="hidden" id="trackingSettingsLoaded" value=""><label class="field">Maximum movement per command (0.5–3.0°)<input id="trackingStepInput" type="number" min="0.5" max="3.0" step="0.1" value="2.3"></label><label class="field">Minimum command interval (0.18–1.0 s)<input id="trackingIntervalInput" type="number" min="0.18" max="1" step="0.01" value="0.20"></label><p class="small muted">Larger movement steps and shorter intervals react faster. Servo settling, target filtering, feedback and motion safety still limit actual speed. Uses original fire-inspection ROS parameters; changes require restarting that controller.</p><div class="metric"><span>Fire Monitoring</span><b id="settingsFireMonitorState">${(state.status.ros || {}).confirmed ? 'CONFIRMED HOLD' : (state.status.ros || {}).fire ? 'ON' : 'OFF'}</b></div><button class="btn blue" onclick="saveTrackingSpeed()">Save Tracking Speed</button><p class="small muted" id="trackingSettingsStatus">Loading ROS tracking parameters…</p></div><div class="card"><h2>Security & Diagnostics</h2><div class="metric"><span>Jetson Power</span><span>${state.status.power_enabled ? 'Enabled' : 'Disabled by server'}</span></div><p class="small muted">Local network: open http://JETSON_IP:8765 from a device on the same Wi-Fi or hotspot.</p></div></div>`; }
function modal(title, content, foot = '') { const root = $('#modalRoot'); root.innerHTML = `<div class="overlay" id="modalBackdrop"><div class="modal" role="dialog" aria-modal="true"><h2>${title}</h2>${content}<div class="dialog-actions">${foot}</div></div></div>`; $('#modalBackdrop').addEventListener('click', e => { if (e.target.id === 'modalBackdrop' && state.mode !== 'qr')
    closeModal(); }); }
function closeModal() { if(typeof restorePatrolCameraPreviews==='function')restorePatrolCameraPreviews(); stopQrArm(); stopQrCamera(); if (qrLiveTimer) {
    clearInterval(qrLiveTimer);
    qrLiveTimer = null;
} if (qrProbeTimer) {
    clearInterval(qrProbeTimer);
    qrProbeTimer = null;
} $('#modalRoot').innerHTML = ''; }
function createMapDialog() { modal('Create New Map', `<label class="field">Map Name<input id="newMapName" placeholder="factory_map_01" maxlength="64" autofocus></label>`, btn('Cancel', 'closeModal()') + btn('Start Mapping', 'beginMap()', 'blue')); }
let mappingStartPending = false;
async function beginMap() { var _a, _b; if (mappingStartPending || ((_b = (_a = state.status.processes) === null || _a === void 0 ? void 0 : _a.mapping) === null || _b === void 0 ? void 0 : _b.running))
    return toast('Mapping already running', true); const name = $('#newMapName').value.trim(); if (!/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(name))
    return toast('Use a safe map name', true); if (state.maps.some(m => m.name === name))
    return toast('Map name already exists', true); mappingStartPending = true; const startBtn = $('#modalRoot .btn.blue'); if (startBtn) {
    startBtn.disabled = true;
    startBtn.textContent = 'Starting…';
} let r; try {
    r = await safe('start_mapping');
}
finally {
    mappingStartPending = false;
    if (startBtn) {
        startBtn.disabled = false;
        startBtn.textContent = 'Start Mapping';
    }
} if (!r)
    return; state.activeMap = name; state.teleop = false; closeModal(); page('maps', 'mapping'); }
function createMissionDialog() { var _a; state.pickMap = (_a = state.maps[0]) === null || _a === void 0 ? void 0 : _a.name; modal('Create Mission', `<label class="field">Mission Name<input id="newMissionName" placeholder="routine_A" maxlength="64"></label><div class="small muted" style="margin-bottom:8px">Select a map</div><div class="modal-map">${state.maps.map(m => `<button class="choice ${m.name === state.pickMap ? 'selected' : ''}" onclick="selectMissionMap('${m.name}')"><img src="/api/maps/${m.name}.png"><span>${esc(m.name)}</span></button>`).join('')}</div>`, btn('Cancel', 'closeModal()') + btn('Create Mission', 'confirmMission()', 'blue')); }
function selectMissionMap(s) { state.pickMap = s; $$('.choice').forEach(x => x.classList.toggle('selected', x.textContent.trim() === s)); }
function confirmMission() { const id = $('#newMissionName').value.trim(); if (!/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(id))
    return toast('Enter a valid mission name', true); if (!state.pickMap)
    return toast('Select a map', true); if (state.missions.some(m => m.id === id))
    return toast('Mission name exists', true); state.mission = { id, name: id, map: state.pickMap, waypoints: [], loop: false }; state.pointIndex = -1; state.localized = false; closeModal(); startEditorLocalization(); }
async function openMission(id) { let m = state.missions.find(x => x.id === id); if (!m)
    return; state.mission = JSON.parse(JSON.stringify(m)); state.pointIndex = -1; state.localized = false; startEditorLocalization(); }
async function initializeLocalization(mapName, pageName) {
    const session=++state.localizationSession;
    state.pose=null;
    state.initialPoseSent=false;
    state.mapMeta=null;
    state.mapImageReady=false;
    state.localizationError=null;
    state.localizationLoading=true;
    state.localizationSubmitting=false;
    state.activeMap=mapName;
    page(pageName,'localize');
    updateLocalizationControls();
    try {
        // Fetch map metadata and start navigation separately; neither result
        // should silently unlock controls before both are ready.
        const loaded=await loadMeta(mapName);
        if (!loaded || state.localizationSession!==session || state.mode!=='localize') return;
        if (!state.status.ros || !state.status.ros.connected)
            await tick();
        if (state.localizationSession!==session || state.mode!=='localize') return;
        const startup=safe('start_navigation',{map:mapName});
        state.localizationStartup=startup;
        const r=await startup;
        if (state.localizationSession!==session || state.mode!=='localize') return;
        if (!r) state.localizationError='Could not start navigation for this map. Use Retry.';
    } finally {
        if (state.localizationSession===session) {
            state.localizationLoading=false;
            state.localizationStartup=null;
            updateLocalizationControls();
        }
    }
}
async function startEditorLocalization() {
    return initializeLocalization(state.mission.map,'mission');
}
async function loadPatrol(id) {
    const m=state.missions.find(x=>x.id===id);
    if(!m) return;
    state.patrolMission=JSON.parse(JSON.stringify(m));
    state.localized=false;
    return initializeLocalization(m.map,'patrol');
}
async function retryLocalizationSetup() {
    if (state.localizationLoading) return;
    return initializeLocalization(state.activeMap,state.page);
}
async function saveMission() { if (!state.mission)
    return; const v = await safe('save_mission', { mission: state.mission }); if (v) {
    state.mission = v;
    await refreshLists();
    toast('Mission saved');
} }
async function deleteMission(id) { if (!confirm('Delete mission ' + id + '?'))
    return; const ok = await safe('delete_mission', { id }); if (ok) {
    await refreshLists();
    render();
} }
async function setToken() { const v = prompt('Jetson dashboard access token:'); if (v === null)
    return; sessionStorage.setItem('stip_token', v.trim()); try {
    const r = await fetch('/api/session', { headers: tokenHeaders(), cache: 'no-store' });
    if (!r.ok)
        throw Error('Invalid token');
    toast('Dashboard access enabled');
    await refreshLists();
    render();
}
catch (e) {
    toast(e.message, true);
} }
async function deleteMap(name) { var _a, _b, _c, _d; const entry = state.maps.find(m => m.name === name); if (!entry)
    return; if (entry.missions)
    return toast('Map is used by ' + entry.missions + ' mission(s). Remove dependencies first.', true); if (((_b = (_a = state.status.processes) === null || _a === void 0 ? void 0 : _a.navigation) === null || _b === void 0 ? void 0 : _b.running) || ((_d = (_c = state.status.processes) === null || _c === void 0 ? void 0 : _c.mapping) === null || _d === void 0 ? void 0 : _d.running))
    return toast('Stop navigation/mapping before deleting a saved map.', true); if (!confirm('Permanently delete map ' + name + ' (.yaml and .pgm)?'))
    return; const r = await safe('delete_map', { name }); if (r) {
    await refreshLists();
    render();
    toast('Map deleted');
} }
function previewMap(name) { modal('Map: ' + esc(name), `<img src="/api/maps/${name}.png" onerror="this.replaceWith(Object.assign(document.createElement('p'),{textContent:'Map preview failed. Check the .pgm file and dashboard logs.'}))" style="width:100%;height:320px;object-fit:contain;background:#081524;border-radius:8px">`, btn('Close', 'closeModal()') + btn('Delete', `closeModal();deleteMap('${name}')`, 'red')); }
function render() { const el = $('#main'); if (state.page === 'home')
    el.innerHTML = `<section class="page">${home()}</section>`;
else if (state.page === 'maps' && state.mode === 'list')
    el.innerHTML = `<section class="page">${maps()}</section>`;
else if (state.page === 'mission' && state.mode === 'list')
    el.innerHTML = `<section class="page">${missions()}</section>`;
else if (state.page === 'patrol' && state.mode === 'list')
    el.innerHTML = `<section class="page">${patrolList()}</section>`;
else if (state.page === 'settings')
    el.innerHTML = `<section class="page">${settings()}</section>`;
else
    renderWorkspace(); }
function fp(id, title, body, classes) { return `<section class="float ${classes}" id="fp_${id}"><div class="floathead" data-drag="${id}"><b>${title}</b><button class="btn s" onclick="minimize('${id}')">−</button></div><div class="floatbody">${body}</div></section>`; }
function minimize(id) { var _a; (_a = $('#fp_' + id)) === null || _a === void 0 ? void 0 : _a.classList.toggle('minimized'); }
function sceneTop() { return `<div class="workspace"><div class="mapback"><div class="mapframe" id="mapframe"><img id="mapImage" alt="Current occupancy grid"><canvas id="mapCanvas"></canvas></div></div><div class="map-status" id="mapStatus">Loading map</div><div class="work-hint" id="mapHint">Wheel: zoom · Right drag: pan · Left drag: rotate view</div><div class="map-zoom">${btn('+', 'adjustZoom(1.15)', 'round')}${btn('−', 'adjustZoom(.87)', 'round')}${btn('⌖', 'resetMapView()', 'round')}</div>`; }
function sceneBottom() { return '</div>'; }
function localizationPanel() {
    const g=localizationGate();
    return fp('localize','Initial Localization',
        `<p class="small muted">Check the robot triangle. If correct, Continue. Otherwise drag on the map to set the location and nose direction, then Localize.</p>`+
        `<div class="metric"><span>Localization</span><span id="locResult" class="${g.error?'bad':g.canContinue?'good':'warntext'}">${esc(g.message)}</span></div>`+
        `<div class="flexwrap" style="margin-top:14px">`+
        `<button class="btn blue" id="localizeStart" onclick="sendInitialPose()" ${g.canLocalize?'':'disabled'}>Localize</button>`+
        `<button class="btn" id="localizeReset" onclick="clearPose()" ${state.localizationLoading?'disabled':''}>Reset</button>`+
        `<button class="btn green" id="localizeContinue" onclick="continueLocalization()" ${g.canContinue?'':'disabled'}>Continue</button>`+
        `<button class="btn s" id="localizationRetry" onclick="retryLocalizationSetup()" style="display:${g.error?'inline-flex':'none'}">Retry</button>`+
        `</div>`, 'right');
}
function mappingTeleopPanel() { return fp('teleop', 'Mapping • Teleop / Save', `<div class="teleop-row"><div><button class="btn ${state.teleop ? 'red' : 'blue'}" onclick="toggleTeleop()">${state.teleop ? 'Stop Teleop' : 'Start Teleop'}</button><div class="small muted" style="margin-top:4px">W A S D · only while held</div></div><div class="dpad"><span></span><button class="btn s" data-move="w">W</button><span></span><button class="btn s" data-move="a">A</button><button class="btn s" data-move="s">S</button><button class="btn s" data-move="d">D</button></div><div class="speeds"><label>Linear <span id="linearValue">${state.linear.toFixed(2)}</span> m/s</label><input type="range" min="0.05" max="0.4" step="0.01" value="${state.linear}" oninput="state.linear=+this.value;$('#linearValue').textContent=this.value"><label>Angular <span id="angularValue">${state.angular.toFixed(2)}</span> rad/s</label><input type="range" min="0.2" max="1.5" step="0.05" value="${state.angular}" oninput="state.angular=+this.value;$('#angularValue').textContent=this.value"></div></div><div class="floating-bottom-actions">${btn('Save Map', 'finishMap(true)', 'green')}${btn('Cancel', 'finishMap(false)', 'red')}</div>`, 'bottom'); }
function missionTeleopPanel() { return fp('missionteleop', 'Drive Robot', `<div class="teleop-row"><div><button class="btn ${state.teleop ? 'red' : 'blue'}" onclick="toggleTeleop()">${state.teleop ? 'Stop Teleop' : 'Enable Teleop'}</button><div class="small muted" style="margin-top:5px">Hold W A S D to drive · release to stop</div></div><div class="dpad"><span></span><button class="btn s" data-move="w">W</button><span></span><button class="btn s" data-move="a">A</button><button class="btn s" data-move="s">S</button><button class="btn s" data-move="d">D</button></div><div class="speeds"><label>Linear <span id="linearValue">${state.linear.toFixed(2)}</span> m/s</label><input type="range" min="0.05" max="0.4" step="0.01" value="${state.linear}" oninput="state.linear=+this.value;$('#linearValue').textContent=this.value"><label>Angular <span id="angularValue">${state.angular.toFixed(2)}</span> rad/s</label><input type="range" min="0.2" max="1.5" step="0.05" value="${state.angular}" oninput="state.angular=+this.value;$('#angularValue').textContent=this.value"></div></div><p class="small muted" style="margin:9px 0 0">Stop the robot before capturing its live map-frame pose.</p>`, 'bottom'); }
function pointList() { var _a; const pts = ((_a = state.mission) === null || _a === void 0 ? void 0 : _a.waypoints) || []; return fp('points', `Waypoints (${pts.length})`, `<div style="max-height:230px;overflow:auto">${pts.map((p, i) => `<div class="item ${state.pointIndex === i ? 'selected' : ''}" onclick="selectPoint(${i})"><span class="symbol">${i + 1}</span><span class="name">${esc(p.name)}</span></div>`).join('') || '<p class="small muted">Drive the robot to a location, then save its current pose.</p>'}</div>${btn('+ Add Current Pose', 'addWaypoint()', 'blue s')}`, 'left'); }
function taskEntry(t, i) { return `<div class="entry"><div class="actions">${t.type === 'qr' ? btn('View Profile', `viewQrTask(${i})`, 's')+btn('Test Inspection', `openQrTestDialog(${i})`, 'blue s')+btn('Gallery', 'openInspectionGallery()', 's') : ''}${btn('↑', `moveTask(${i},-1)`, 's')}${btn('↓', `moveTask(${i},1)`, 's')}${btn('×', `removeTask(${i})`, 'red s')}</div><b>${i + 1}. ${t.type === 'qr' ? 'QR Scan' : 'Pause ' + t.seconds + 's'}</b><div class="small muted">${t.type === 'qr' ? esc(t.profile) : 'Wait before next task'}</div></div>`; }
function selectedPointPanel() { var _a, _b, _c; let pt = (_b = (_a = state.mission) === null || _a === void 0 ? void 0 : _a.waypoints) === null || _b === void 0 ? void 0 : _b[state.pointIndex]; const got = ((_c = state.status.ros) === null || _c === void 0 ? void 0 : _c.robot) || {}; return fp('selected', 'Selected Waypoint', pt ? `<label class="field">Name<input id="pointName" value="${esc(pt.name)}" onchange="updatePointName(this.value)"></label><div class="small muted">X ${pt.x.toFixed(2)} · Y ${pt.y.toFixed(2)} · Yaw ${(pt.yaw * 180 / Math.PI).toFixed(1)}°</div><div class="flexwrap" style="margin:12px 0">${btn('Navigate Here', 'navigateSelected()', 'blue')}${btn('Delete', 'deletePoint()', 'red s')}</div><b class="small">Tasks in execution order</b>${pt.tasks.map(taskEntry).join('') || '<div class="small muted" style="padding:10px 0">No task: continue to next waypoint.</div>'}<div class="flexwrap" style="margin-top:10px">${btn('+ QR Task', 'openQrEditor()', 'blue s')}${btn('+ Pause', 'addPause()', 's')}</div>` : `<p class="small muted">Select or add a waypoint to edit its task sequence.</p>`, 'right'); }
function armControlPanel() { var _a, _b, _c; return fp('arm', 'QR Task • Camera Arm', `<div class="metric"><span>Pan</span><div class="row">${btn('−', `armNudge('pan',-2)`, 's')}<b id="panAngle">${state.arm.pan.toFixed(0)}°</b>${btn('+', `armNudge('pan',2)`, 's')}</div></div><div class="metric"><span>Tilt</span><div class="row">${btn('−', `armNudge('tilt',-2)`, 's')}<b id="tiltAngle">${state.arm.tilt.toFixed(0)}°</b>${btn('+', `armNudge('tilt',2)`, 's')}</div></div><label class="field" style="margin-top:12px">Profile Name<input id="profileName" value="QR_${esc(((_c = (_b = (_a = state.mission) === null || _a === void 0 ? void 0 : _a.waypoints) === null || _b === void 0 ? void 0 : _b[state.pointIndex]) === null || _c === void 0 ? void 0 : _c.id) || 'P1')}"></label>${btn('Open QR Enrollment', 'openQrDialog()', 'blue')}<p class="small muted">Enrollment saves the actual current arm angles with the QR and indicator ROI.</p>`, 'right'); }
function cameraPanel() { var _a; return fp('camera', 'RGB • Captured ROI', `<div class="camera" id="captureFrame" style="height:220px"><img id="captureImg" alt="Captured QR frame" src="${((_a = state.capture) === null || _a === void 0 ? void 0 : _a.jpeg) || '/api/camera'}"><div class="roi" id="roiBox" style="display:none"></div></div><div class="small muted">Draw the indicator rectangle on the captured image.</div><div class="flexwrap" style="margin-top:9px">${btn('Enroll QR', 'enrollQr()', 'green')}${btn('Cancel QR', 'closeQrEditor()')}</div>`, 'lowerleft wide'); }
function mappingLeft() { var _a, _b, _c, _d, _e, _f; let r = state.status.ros || {}; return fp('mapping', 'Mapping Status', `<div class="metric"><span>Map</span><b>${esc(state.activeMap)}</b></div><div class="metric"><span>SLAM</span><b class="${((_b = (_a = state.status.processes) === null || _a === void 0 ? void 0 : _a.mapping) === null || _b === void 0 ? void 0 : _b.running) ? 'good' : 'warntext'}">${((_d = (_c = state.status.processes) === null || _c === void 0 ? void 0 : _c.mapping) === null || _d === void 0 ? void 0 : _d.running) ? 'RUNNING' : 'WAITING'}</b></div><div class="metric"><span>LiDAR</span><b class="${((_e = r.health) === null || _e === void 0 ? void 0 : _e.lidar) ? 'good' : 'warntext'}">${((_f = r.health) === null || _f === void 0 ? void 0 : _f.lidar) ? 'LIVE' : 'WAITING'}</b></div><div class="metric"><span>Robot TF</span><b>${r.robot_map ? 'AVAILABLE' : 'WAITING'}</b></div>`, 'left'); }
function missionTop() { var _a, _b; return fp('mission', 'Mission', `<div class="metric"><span>Name</span><b>${esc((_a = state.mission) === null || _a === void 0 ? void 0 : _a.name)}</b></div><div class="metric"><span>Map</span><b>${esc((_b = state.mission) === null || _b === void 0 ? void 0 : _b.map)}</b></div><div class="flexwrap" style="margin-top:11px">${btn('Save Mission', 'saveMission()', 'green s')}${btn('Exit Editor', 'exitMission()', 's')}</div>`, 'lowerleft'); }
function patrolControlPanel() { var _a, _b; const m = state.patrolMission, ms = ((_a = state.status.ros) === null || _a === void 0 ? void 0 : _a.mission) || {}; return fp('patrol', 'Patrol Control', `<div class="metric"><span>Mission</span><b>${esc((m === null || m === void 0 ? void 0 : m.name) || '')}</b></div><div class="metric"><span>Progress</span><b>${ms.index || 0} / ${((_b = m === null || m === void 0 ? void 0 : m.waypoints) === null || _b === void 0 ? void 0 : _b.length) || 0}</b></div><div class="metric"><span>State</span><b>${esc(ms.state || 'READY')}</b></div><div class="flexwrap" style="margin:11px 0">${btn('Start', 'startPatrol()', 'green')}${btn('Pause', 'patrolCmd(\'patrol_pause\')', 'warn')}${btn('Resume', 'patrolCmd(\'patrol_resume\')', 'blue')}${btn('Stop', 'patrolCmd(\'stop\')', 'red')}</div><label class="row small"><input type="checkbox" ${(m === null || m === void 0 ? void 0 : m.loop) ? 'checked' : ''} onchange="state.patrolMission.loop=this.checked"> Loop</label><label class="row small"><input type="checkbox" ${state.fire ? 'checked' : ''} data-fire-monitor="true" onchange="fireToggle(this.checked)"> Fire Monitoring</label><label class="row small"><input type="checkbox" ${state.qr ? 'checked' : ''} onchange="qrToggle(this.checked)"> QR Tasks</label>`, 'left'); }
function patrolMonitorPanel() {
    const r=state.status.ros || {}, arm=r.arm || {};
    const fireStatus=r.confirmed?'CONFIRMED':r.warning?'SUSPECTED':r.fire?'MONITORING':'OFF';
    return fp('monitor','Live Inspection',`
        <div class="camera patrol-rgb"><div class="live-inspection-image"><img id="liveCamera" src="/api/camera.mjpg" alt="RGB inspection camera"><svg id="liveInspectionOverlay" class="inspection-overlay" preserveAspectRatio="xMidYMid meet" aria-label="QR and fire detection boxes"></svg></div></div>
        <div class="flexwrap patrol-preview-actions">${btn('Expand Live Camera','openExpandedCamera()','blue s')}</div>
        <div class="camera patrol-thermal"><img id="liveThermal" src="/api/thermal.mjpg" alt="Thermal camera"></div>
        <div class="metric"><span>Fire</span><b id="patrolFireStatus" class="${r.confirmed?'bad':r.warning?'warntext':'good'}">${fireStatus}</b></div>
        <div class="small muted" id="patrolFireDetail" style="margin:4px 0 8px"></div>
        <div class="metric"><span>Auto Tracking</span><b id="fireTrackingStatus">—</b></div>
        <div class="small muted" id="fireTrackingDetail" aria-live="polite" style="margin:3px 0 8px"></div>
        <div class="metric"><span>Target temperature</span><b id="fireTargetTemperature">—</b></div>
        <div class="metric"><span>Gas raw / Δ</span><b id="patrolGasStatus">${r.gas_raw===undefined?'—':esc(r.gas_raw)} / ${r.gas_change_percent===undefined?'—':esc(r.gas_change_percent)+'%'}</b></div>
        <div class="metric"><span>Navigation</span><b id="patrolNavStatus">${esc(r.nav||'—')}</b></div>
        <div class="metric"><span>Arm pan / tilt</span><b id="patrolArmStatus">${Number(arm.pan||0).toFixed(0)}° / ${Number(arm.tilt||0).toFixed(0)}°</b></div>
        <div id="fireAcknowledge">${r.warning||r.confirmed?btn('Acknowledge Fire','fireClear()','warn s'):''}</div>`, 'right patrol-monitor');
}
function patrolActivityPanel() {
    return fp('activity','Patrol Activity & Evidence',`
        <div class="patrol-activity-toolbar">
            ${btn('Inspection Gallery','openInspectionGallery()','blue s')}
            ${btn('Fire Evidence','openFireGallery()','s')}
        </div>
        <div class="metric"><span>Inspection results</span><b id="inspectionResultCount">0</b></div>
        <div id="inspectionResults" class="inspection-results patrol-activity-results">No inspections yet.</div>
        <div class="metric"><span>Navigation / Events</span><span class="small muted">Latest first</span></div>
        <div id="patrolEventLog" class="patrol-event-log" role="log" aria-label="Patrol navigation and inspection events">Waiting for patrol events…</div>`, 'lowerleft patrol-activity');
}

function renderWorkspace() { let ui = sceneTop(), title = ''; if (state.page === 'maps' && state.mode === 'mapping') {
    title = 'Mapping';
    ui += mappingLeft() + mappingTeleopPanel();
}
else if (state.mode === 'localize') {
    title = 'Set Initial Pose';
    ui += localizationPanel();
}
else if (state.page === 'mission') {
    title = 'Mission Editor';
    ui += pointList() + selectedPointPanel() + missionTop() + (state.mode === 'edit' ? missionTeleopPanel() : '');
}
else if (state.page === 'patrol') {
    title = 'Patrol';
    ui += patrolControlPanel() + patrolMonitorPanel() + patrolActivityPanel();
} ui += sceneBottom(); $('#main').innerHTML = ui; $('#mapStatus').textContent = title + (state.activeMap ? ' · ' + state.activeMap : ''); loadImageIntoMap(); attachMapInteractions(); attachFloatDrag(); attachTeleopButtons(); if(state.page==='patrol' && state.mode==='live')warmRegisteredOverlay(); }
