// STIP dashboard module: runtime
async function exitMission() {requestLeaveMission('mission','list');}
async function startPatrol() { if(state.teleop)return toast('Release manual control before starting Patrol',true); if (!state.patrolMission)
    return; if (!state.localized) return toast('Confirm the robot pose before starting patrol', true); state.patrolMission.loop = !!state.patrolMission.loop; const r = await safe('patrol_start', { mission_id: state.patrolMission.id, loop: state.patrolMission.loop, operator_confirmed: true }); if (r)
    toast('Patrol start accepted'); }
async function patrolCmd(cmd) { if(state.teleop && cmd!=='stop')return toast('Release manual teleop before continuing Patrol',true); if(cmd==='stop' && state.teleop){releaseTeleop();state.teleop=false;teleopSession=null;await safe('teleop_arm',{enabled:false});render();} const r = await safe(cmd); if (r)
    toast('Command sent: ' + cmd); }
async function qrToggle(enabled) { const r = await safe('qr_enable', { enabled }); if (r)
    state.qr = enabled; }
async function fireClear() { if (!confirm('Acknowledge fire? Confirm the scene is clear and the robot remains safely stopped.'))
    return; const r = await safe('fire_clear'); if (r)
    toast('Fire cleared by inspection controller; patrol remains stopped'); }
// One operator control: the fire monitor checkbox also requests the ROS
// fire-inspection auto-aim capability. The original controller decides when
// actual arm motion is permitted (stationary/paused acknowledgement required).
async function fireToggle(enabled) {
    const control=$('#fp_patrol input[data-fire-monitor]');
    if(control)control.disabled=true;
    try {
        if(enabled) {
            const started=await safe('fire_inspection_start');
            if(!started)throw Error('Fire inspection stage did not start');
        }
        const status=await safe('fire_enable',{enabled});
        if(!status)throw Error('Fire monitoring change was rejected');
        state.fire=enabled;
        state.fireTrackingPending=enabled;
        if(enabled){
            const result=await requestAutomaticFireTracking();
            if(!result)toast('Fire monitoring ON. Tracking is waiting for a stationary/paused ROS acknowledgement.');
            else toast('Fire monitoring ON · tracking requested through ROS controller');
        }else {
            state.fireTrackingPending=false;
            try{await act('fire_tracking',{enabled:false},3500);}catch(e){toast('Monitoring OFF; tracking disable not acknowledged: '+e.message,true);}
            toast('Fire monitoring OFF (confirmed-fire safety holds remain in force)');
        }
    }catch(e){toast(e.message,true);state.fire=!enabled;}
    finally {
        if(control){control.disabled=false;control.checked=!!state.fire;}
    }
}
let trackingRequestBusy=false, trackingRetryAt=0;
async function requestAutomaticFireTracking(){
    if(trackingRequestBusy || !state.fire)return false;
    trackingRequestBusy=true;
    try {
        const result=await act('fire_tracking',{enabled:true},4000);
        state.fireTrackingPending=false;
        return !!(result && result.enabled);
    }catch(e) {
        state.fireTrackingPending=true;
        trackingRetryAt=Date.now()+5000;
        return false;
    }finally{trackingRequestBusy=false;}
}
async function restartStage(s) { if (!confirm('Restart stage ' + s + ' using stage manager?'))
    return; const r = await safe('stage_restart', { stage: s }); if (r)
    toast('Restart requested: ' + s); }
async function systemStart() { const r = await safe('start_system'); if (r)
    toast('System launch started; waiting for readiness'); }
async function systemStop() { if (!confirm('Stop managed ROSMASTER system?'))
    return; await safe('stop_system'); }
function powerAction(kind) { if (!confirm((kind === 'power_restart' ? 'RESTART' : 'POWER OFF') + ' Jetson? Robot will stop and dashboard will disconnect.'))
    return; safe(kind); }
async function emergencyStop() { await safe('stop'); toast('STOP requested — verify physical robot has stopped'); }
function attachFloatDrag() { $$('.floathead').forEach(head => { let parent = head.parentElement, origin = null; head.addEventListener('pointerdown', e => { if (parent.classList.contains('patrol-monitor') || parent.classList.contains('patrol-activity')) return; if (e.target.closest('button'))
    return; const bounds = parent.getBoundingClientRect(), base = $('.workspace').getBoundingClientRect(); origin = { x: e.clientX, y: e.clientY, left: bounds.left - base.left, top: bounds.top - base.top }; head.setPointerCapture(e.pointerId); parent.style.transform = 'none'; parent.style.left = origin.left + 'px'; parent.style.top = origin.top + 'px'; parent.style.right = 'auto'; parent.style.bottom = 'auto'; }); head.addEventListener('pointermove', e => { if (!origin)
    return; const base = $('.workspace'); let x = Math.max(0, Math.min(base.clientWidth - parent.offsetWidth, origin.left + e.clientX - origin.x)), y = Math.max(0, Math.min(base.clientHeight - parent.offsetHeight, origin.top + e.clientY - origin.y)); parent.style.left = x + 'px'; parent.style.top = y + 'px'; }); head.addEventListener('pointerup', () => origin = null); head.addEventListener('pointercancel', () => origin = null); }); }
let lastSeen = {}, tickBusy = false;
async function tick() {
    var _a, _b, _c, _d, _e, _f, _g, _h;
    if (tickBusy)
        return;
    tickBusy = true;
    try {
        state.status = await get('status');
        const connected = !!((_a = state.status.ros) === null || _a === void 0 ? void 0 : _a.connected);
        const connection = $('#connection');
        if (connection) {
            connection.textContent = connected ? '● ROS connected' : '● ROS offline · Dashboard online';
            connection.style.color = connected ? '#35d6a2' : '#ffba72';
        }
        const engine = $('#engine');
        if (engine)
            engine.textContent = 'Engine: ' + (state.status.engine || 'stip').toUpperCase();
        const clock = $('#timestamp');
        if (clock)
            clock.textContent = new Date().toLocaleTimeString();
        if ((_b = state.status.ros) === null || _b === void 0 ? void 0 : _b.arm)
            state.arm = state.status.ros.arm;
        if (state.page === 'home') {
            const health = $('#componentHealth');
            if (health)
                health.innerHTML = healthTable() + auxHealth();
            const logs = $('#eventList');
            if (logs) {
                const events = (state.status.events || []).slice(-35).reverse(), key = events.join('\u0001');
                if (lastSeen.logs !== key) {
                    lastSeen.logs = key;
                    logs.innerHTML = events.map(e => `<div class="logline">${esc(e)}</div>`).join('') || '<div class="small muted">No events recorded</div>';
                }
            }
        }
        updateGasAlarmIndicator();
        updateLocalizationControls();
        // Reconcile *all* teleop contexts (Mission, Mapping, Patrol), not only
        // Patrol. Another browser, an emergency STOP, or a ROS restart may
        // revoke this session. Never keep showing an armed control after that.
        const liveTeleop=(state.status.ros||{}).teleop;
        if(state.teleop && (state.status.ros||{}).connected && liveTeleop &&
            !teleopChanging && Date.now()-(state.teleopStartAt||0)>1100 &&
            (liveTeleop.armed===false || (liveTeleop.session!==undefined &&
                teleopSession!==null && liveTeleop.session!==teleopSession))){
            releaseTeleop();state.teleop=false;teleopSession=null;
            render();toast('ROS revoked Teleop control (STOP, another session, or adapter restart). Re-enable to drive.',true);
        }
        if (state.page==='settings') {
            const fireState=$('#settingsFireMonitorState');
            if(fireState){
                const ros=state.status.ros||{};
                fireState.textContent=ros.confirmed?'CONFIRMED HOLD':ros.fire?'ON':'OFF';
                fireState.className=ros.confirmed?'bad':ros.fire?'warntext':'good';
            }
            const g=$('#gasCalibration');
            if(g) g.innerHTML=gasCalibrationPanel();
            const config=$('#fireSettingsLoaded');
            if(config && !config.value)loadFireSettings();
            const gasConfig=$('#gasAlarmSettingsLoaded');
            if(gasConfig && !gasConfig.value)loadGasAlarmSettings();
            const gasState=$('#gasAlarmSettingsState');
            if(gasState)gasState.textContent=formatGasAlarmStatus();
            const trackConfig=$('#trackingSettingsLoaded');
            if(trackConfig && !trackConfig.value)loadTrackingSpeed();
        }
        if (state.page==='patrol' && state.mode==='live') {
            const gasPatrol=$('#patrolGasAlarm');if(gasPatrol)gasPatrol.textContent=formatGasAlarmStatus();
            const teleopStatus=$('#patrolManualStatus');if(teleopStatus)teleopStatus.textContent=state.teleop?'MANUAL CONTROL ARMED — hold to move':'Autonomous mode / idle';
            updatePatrolInspection();
            const fusion=(state.status.ros || {}).fire_fusion || {};
            if(state.fireTrackingPending && state.fire && fusion.navigation_pause_ack && !trackingRequestBusy && Date.now()>=trackingRetryAt) {
                trackingRetryAt=Date.now()+5000;
                requestAutomaticFireTracking();
            }
        }
        const mapMode = state.page === 'maps' && state.mode === 'mapping';
        if (mapMode && connected && state.status.map) {
            state.mapMeta = state.status.map;
            const version = state.status.map.version || state.status.map_version || 0;
            if (version && latestRenderedMap !== version) {
                latestRenderedMap = version;
                const img = $('#mapImage');
                if (img) {
                    img.src = '/api/live_map?version=' + encodeURIComponent(version);
                }
            }
        }
        const laser = $('#mapCanvas');
        if (laser)
            paintMap();
        if (mapMode) {
            const s = ((_e = state.status.ros) === null || _e === void 0 ? void 0 : _e.teleop) || {};
            const hint = $('#mapHint');
            if (hint)
                hint.textContent = 'Wheel: zoom · Right drag: pan · Left drag: rotate view';
        }
        else {
            const hint = $('#mapHint');
            if (hint)
                hint.textContent = state.mode === 'localize' ? 'Left drag: robot position and heading · Right drag: pan · Wheel: zoom' : 'Drive robot, then Add Current Pose · Right drag: pan · Wheel: zoom';
        }
        // Thermal preview stays on the same MJPEG connection. Do not replace its
        // src every 1.5 seconds; that caused the apparent low display FPS.
        const now = Date.now();
        const camera = $('#liveCamera');
        if (camera && connected && !state.expandedCameraOpen && !camera.src.includes('/api/camera.mjpg') && now - state.latestCamVer > 900) {
            state.latestCamVer = now;
            camera.src = '/api/camera?t=' + now;
        }
        if (state.page === 'patrol' && state.mode === 'live') {
            const p = $('#fp_patrol'), v = ((_f = state.status.ros) === null || _f === void 0 ? void 0 : _f.mission) || {};
            if (p === null || p === void 0 ? void 0 : p.querySelector('.floatbody')) {
                const a = p.querySelector('.metric:nth-child(2) b'), b = p.querySelector('.metric:nth-child(3) b');
                if (a)
                    a.textContent = (v.index || 0) + ' / ' + (((_h = (_g = state.patrolMission) === null || _g === void 0 ? void 0 : _g.waypoints) === null || _h === void 0 ? void 0 : _h.length) || 0);
                if (b)
                    b.textContent = v.state || 'READY';
            }
        }
    }
    catch (e) {
        const el = $('#connection');
        if (el) {
            el.textContent = '● Dashboard API unavailable';
            el.style.color = '#fba56b';
        }
    }
    finally {
        tickBusy = false;
    }
}
window.addEventListener('resize', paintMap);
window.addEventListener('keydown', e => { var _a; if (e.key === 'Escape') {
    if (state.mode === 'qr')
        closeQrEditor();
    else
        closeModal();
    return;
} if (state.mode === 'qr' && $('#modalBackdrop') && !e.repeat && !e.altKey && !e.ctrlKey && !e.metaKey && !['INPUT', 'TEXTAREA', 'SELECT'].includes((_a = document.activeElement) === null || _a === void 0 ? void 0 : _a.tagName)) {
    const k = e.key.toLowerCase();
    if ('wasd'.includes(k) && k.length === 1) {
        e.preventDefault();
        startQrArm(k);
    }
} });
(async () => { if (location.hostname !== '127.0.0.1' && location.hostname !== 'localhost') {
    let r = await fetch('/api/session', { headers: tokenHeaders() }).catch(() => null);
    if (r && r.status === 401)
        await setToken();
}
else {
    await fetch('/api/session', { headers: tokenHeaders() }).catch(() => null);
} await refreshLists(); render(); await tick(); setInterval(tick, 400); })();
// Release control regardless of where the pointer/keyboard focus moves.
window.addEventListener('keyup', e => {
    if (state.mode === 'qr' && e.key.toLowerCase() === qrHeld)
        stopQrArm();
});
window.addEventListener('blur', () => { stopQrArm(); releaseTeleop(); });
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        stopQrArm();
        releaseTeleop();
    }
});
