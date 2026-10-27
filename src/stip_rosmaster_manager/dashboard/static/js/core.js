// STIP dashboard module: core
const $ = (s, root = document) => root.querySelector(s), $$ = (s, root = document) => [...root.querySelectorAll(s)];
const esc = s => String(s !== null && s !== void 0 ? s : '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const state = { page: 'home', mode: 'list', status: { ros: { connected: false } }, maps: [], missions: [], activeMap: null, mapMeta: null, mission: null, pointIndex: -1, pose: null, localized: false, initialPoseSent: false, localizationLoading: false, localizationSubmitting: false, localizationSession: 0, localizationStartup: null, localizationError: null, mapImageReady: false, arm: { pan: 90, tilt: 0 }, teleop: false, held: null, linear: .18, angular: .65, fire: false, qr: true, loop: false, patrolMission: null, roi: null, capture: null, roiDragging: false, latestMapVer: 0, latestCamVer: 0, latestThermalVer: 0 };
let qrLiveTimer = null, qrProbeTimer = null, qrProbeBusy = false, qrNudgeBusy = false;
let qrHeld = null, qrArmLoop = null, qrCamActive = false, qrCamBusy = false, qrCamGeneration = 0;
let teleopSending = false, teleopStopPending = false, teleopChanging = false, patrolExiting = false;
let toastTimer, moveTimer, zoom = 1, dragMove = null, paintCtx = null, curMapName = null, teleopSeq = 0, mapView = { x: 0, y: 0, angle: 0 }, latestRenderedMap = 0;
function toast(msg, isError = false) { const root = $('#toastRoot'); root.innerHTML = `<div class="toast ${isError ? 'error' : ''}">${esc(msg)}</div>`; clearTimeout(toastTimer); toastTimer = setTimeout(() => root.innerHTML = '', 4700); }
async function get(path) { const r = await fetch('/api/' + path, { headers: tokenHeaders(), cache: 'no-store' }); const x = await r.json(); if (!r.ok)
    throw Error(x.error || r.statusText); return x; }
function tokenHeaders() { let token = ''; try {
    token = sessionStorage.getItem('stip_token') || '';
}
catch (e) { } return { 'X-Dashboard-Token': token }; }
// Bounded actions: even if the bridge/API stalls, never trap the operator.
// Keep a timer independent of AbortController for older browsers.
async function act(action, payload = {}, deadlineMs = 0) {
    const ctl = (deadlineMs && typeof AbortController !== 'undefined') ? new AbortController() : null;
    let timer = null;
    const work = (async () => {
        const options = { method: 'POST', headers: { 'Content-Type': 'application/json', ...tokenHeaders() }, body: JSON.stringify({ action, payload }) };
        if (ctl) options.signal = ctl.signal;
        const r = await fetch('/api/action', options);
        const x = await r.json();
        if (!r.ok || !x.ok) throw Error(x.error || 'Action rejected');
        return x.result;
    })();
    if (!(deadlineMs > 0)) return work;
    const timeout = new Promise((resolve, reject) => {
        timer = setTimeout(() => {
            if (ctl) ctl.abort();
            reject(Error('Dashboard request timed out; check server status.'));
        }, deadlineMs);
    });
    try {
        return await Promise.race([work, timeout]);
    } catch(e) {
        if (e && e.name === 'AbortError') throw Error('Dashboard request timed out; check server status.');
        throw e;
    } finally {
        if (timer !== null) clearTimeout(timer);
    }
}
async function safe(action, payload = {}, onSuccess) { try {
    const x = await act(action, payload);
    if (onSuccess)
        onSuccess(x);
    return x;
}
catch (e) {
    toast(e.message, true);
    return null;
} }
async function refreshLists() { try {
    const [a, b] = await Promise.all([get('maps'), get('missions')]);
    state.maps = a.maps;
    state.missions = b.missions;
}
catch (e) {
    toast(e.message, true);
} }
function btn(text, on, cls = '') { return `<button class="btn ${cls}" onclick="${on}">${text}</button>`; }
function page(name, mode) { if ((typeof patrolExiting !== 'undefined' && patrolExiting) || (typeof localizationExiting !== 'undefined' && localizationExiting) || (typeof missionExiting !== 'undefined' && missionExiting)) return;
    if (state.page === 'mission' && (state.mode === 'edit' || state.mode === 'qr') &&
        (name !== 'mission' || (mode || 'list') !== state.mode)) {
        requestLeaveMission(name,mode);
        return;
    }
    if (state.page === 'patrol' && state.mode === 'live' && (name !== 'patrol' || (mode || 'list') !== 'live')) {
        requestLeavePatrol(name, mode);
        return;
    }
    if (state.mode === 'localize' && (name !== state.page || (mode || 'list') !== 'localize')) {
    requestLeaveLocalization(name, mode);
    return;
} if (state.page === 'maps' && state.mode === 'mapping' && (name !== 'maps' || (mode && mode !== 'mapping'))) {
    requestLeaveMapping(name, mode);
    return;
} const wasArmed = state.teleop; releaseTeleop(); if (wasArmed) {
    state.teleop = false;
    safe('teleop_arm', { enabled: false });
} zoom = 1; mapView = { x: 0, y: 0, angle: 0 }; latestRenderedMap = 0; state.page = name; state.mode = mode || 'list'; $('#sidebar').querySelectorAll('.nav').forEach(n => n.classList.toggle('active', n.dataset.page === name)); render(); }
$$('.nav').forEach(n => n.addEventListener('click', () => { page(n.dataset.page); }));
function requestLeaveMission(name,mode) {
    if (state.mode==='qr') {
        stopQrArm(); // No held servo commands when operator opens a dialog
    }
    modal('Exit Mission Editor?',
        '<p>Unsaved mission edits will be lost. Stop manual teleop and any dashboard-owned navigation before leaving?</p>',
        btn('Stay Here','closeModal()')+
        btn('Stop Navigation & Leave',`exitMissionAndLeave('${name}','${mode||'list'}')`,'red'));
}
let missionExiting=false;
async function exitMissionAndLeave(name,mode) {
    if (missionExiting || state.page!=='mission') return false;
    missionExiting=true;
    stopQrArm();
    releaseTeleop();
    try {
        const result=await act('exit_mission',{},10000);
        if (!result || !result.stopped || !result.navigation_stopped)
            throw Error('Mission/navigation shutdown was not confirmed');
        stopQrCamera();
        closeModal();
        state.mission=null;state.pointIndex=-1;state.capture=null;state.roi=null;
        state.localized=false;state.mode='list';state.teleop=false;
        missionExiting=false;
        page(name,mode);
        return true;
    } catch(e) {
        toast('Mission exit blocked: '+e.message,true);
        return false;
    } finally {missionExiting=false;}
}
function requestLeavePatrol(name, mode) {
    // The Patrol page may own move_base and a running mission. Never discard
    // the screen while those processes are still active.
    modal('Exit Patrol?',
        '<p>Stop the patrol, cancel robot motion and shut down dashboard-owned navigation before leaving?</p>',
        btn('Stay on Patrol', 'closeModal()') +
        btn('Stop Patrol & Leave', `exitPatrolAndLeave('${name}','${mode || 'list'}')`, 'red'));
}
async function exitPatrolAndLeave(name, mode) {
    if (patrolExiting || state.page !== 'patrol' || state.mode !== 'live') return;
    patrolExiting=true;
    const exitButton=$('#modalRoot .btn.red');
    if (exitButton) {exitButton.disabled=true;exitButton.textContent='Stopping patrol…';}
    try {
        const result=await act('exit_patrol',{},12000);
        if (!result || !result.stopped || !result.navigation_stopped ||
            result.fire_monitoring_off !== true || result.tracking_off !== true)
            throw Error('Patrol, navigation or Fire Monitoring shutdown not confirmed');
        // Preserve a confirmed ROS fire alarm on the server; the backend
        // refuses exit while it remains active. Reset only operator UI state.
        state.fire=false;
        state.fireTrackingPending=false;
        trackingRetryAt=0;
        closeModal();
        registeredOverlayReady=false;registeredOverlayPromise=null;
        state.localized=false;
        state.patrolMission=null;
        state.mode='list'; // bypass only the patrol guard after verified shutdown
        patrolExiting=false;
        page(name,mode);
    } catch(e) {
        toast('Cannot leave Patrol safely: '+e.message,true);
        if (exitButton) {exitButton.disabled=false;exitButton.textContent='Retry Stop & Leave';}
    } finally {patrolExiting=false;}
}
function requestLeaveLocalization(name, mode) {
    // Sidebar does not silently abandon the localization session.
    modal('Exit localization?',
        '<p>Leave the localization screen and stop dashboard-managed navigation? Unsaved mission edits will be lost.</p>',
        btn('Stay Here', 'closeModal()') +
        btn('Exit Localization', `exitLocalizationAndLeave('${name}','${mode || 'list'}')`, 'red'));
}
let localizationExiting=false;
async function exitLocalizationAndLeave(name, mode) {
    if (state.mode !== 'localize' || localizationExiting) return;
    localizationExiting=true;
    const oldSession=state.localizationSession;
    state.localizationSession=oldSession+1; // invalidate in-flight render callbacks
    const startup=state.localizationStartup;
    const exitButton=$('#modalRoot .btn.red');
    if(exitButton){exitButton.disabled=true;exitButton.textContent='Stopping navigation…';}
    try {
        // Wait for an in-flight start to complete BEFORE final stop. If start
        // hangs indefinitely, abort exit instead of showing a false stop.
        if (startup) {
            await Promise.race([
                startup,
                new Promise((_,reject)=>setTimeout(()=>reject(Error('Navigation startup still pending')),6500))
            ]);
        }
        const result=await act('stop_navigation',{},8000);
        if (!result || !result.stopped) throw Error('Navigation stop was not confirmed');
        closeModal();
        state.localizationLoading=false;
        state.localizationSubmitting=false;
        state.localizationError=null;
        state.localized=false;
        state.initialPoseSent=false;
        state.pose=null;
        state.mapMeta=null;
        state.mapImageReady=false;
        state.activeMap=null;
        state.localizationStartup=null;
        state.mode='list';
        localizationExiting=false;
        page(name,mode);
    } catch(e) {
        state.localizationError='Navigation cleanup not confirmed: '+e.message;
        toast(state.localizationError,true);
        if(exitButton){exitButton.disabled=false;exitButton.textContent='Retry Stop & Leave';}
        updateLocalizationControls();
    } finally {localizationExiting=false;}
}

function requestLeaveMapping(name, mode) { modal('Mapping is active', '<p>Stop mapping and discard the unsaved map before leaving?</p>', btn('Keep Mapping', 'closeModal()') + btn('Stop Mapping & Leave', `abortAndLeave('${name}','${mode || 'list'}')`, 'red')); }
async function abortAndLeave(name, mode) { closeModal(); releaseTeleop(); await safe('teleop_arm', { enabled: false }); const result = await safe('stop_mapping'); if (!result || !result.stopped)
    return toast('Cannot confirm mapping stopped; staying here', true); state.mode = 'list'; state.teleop = false; await refreshLists(); page(name, mode); }
function head(title, description, actions = '') { return actions ? `<div class="section-head compact-head">${actions}</div>` : ''; }
function healthTable() { var _a; const modules = ((_a = state.status.ros) === null || _a === void 0 ? void 0 : _a.modules) || {}; const names = ['bringup', 'camera', 'qr', 'fire', 'gas', 'thermal', 'ui']; return names.map(n => { let m = modules[n] || {}, ok = m.state === 'READY'; return `<div class="metric"><span><span class="${ok ? 'good' : m.state === 'BLOCKED' ? 'bad' : 'warntext'}">●</span> ${esc(n.toUpperCase())}</span><span class="small muted" title="${esc(m.detail || '')}">${esc(m.state || 'UNKNOWN')}</span><button class="btn s" ${!['ERROR', 'BLOCKED', 'DEGRADED'].includes(m.state) ? 'disabled' : ''} onclick="restartStage('${n}')">Restart</button></div>`; }).join(''); }
function auxHealth() { var _a, _b, _c; const r = state.status.ros || {}, p = state.status.processes || {}; return `<div class="metric"><span>● LIDAR</span><b class="${((_a = r.health) === null || _a === void 0 ? void 0 : _a.lidar) ? 'good' : 'warntext'}">${((_b = r.health) === null || _b === void 0 ? void 0 : _b.lidar) ? 'LIVE' : 'NO SCAN'}</b></div><div class="metric"><span>● NAVIGATION</span><b>${((_c = p.navigation) === null || _c === void 0 ? void 0 : _c.running) ? 'ACTIVE' : 'IDLE'}</b></div>`; }
