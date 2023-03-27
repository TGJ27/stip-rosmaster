// STIP dashboard module: mission
// Localization readiness is a UI/workflow state, NOT an AMCL covariance gate.
// An existing map-frame pose can be accepted directly by the operator.
// A newly dragged draft MUST be sent to /initialpose before Continue.
function localizationGate() {
    const ros = state.status.ros || {};
    const proc = state.status.processes || {};
    const hasMap = !!(state.mapMeta && state.mapMeta.width && state.mapMeta.height && state.mapMeta.resolution);
    const nav = !!(proc.navigation && proc.navigation.running);
    const imageReady = !!state.mapImageReady;
    const amclSubscriber = !!(ros.localization && ros.localization.initialpose_subscribers);
    const connected = !!ros.connected;
    const busy = !!(state.localizationLoading || state.localizationSubmitting);
    // Continue is an operator-only UI transition. Do not require a second
    // ROS acknowledgement or an AMCL covariance-ready flag when a map pose
    // is already visible. The localization setup request must still finish.
    const ready = hasMap && imageReady && connected && !busy;
    const robot = ros.robot_map;
    const hasPose = !!(robot && [robot.x, robot.y, robot.yaw].every(Number.isFinite));
    const unsentDraft = !!(state.pose && !state.initialPoseSent);
    let message = '';
    if (state.localizationError) message = state.localizationError;
    else if (state.localizationLoading) message = 'Loading saved map and starting navigation…';
    else if (!hasMap) message = 'Waiting for saved map metadata…';
    else if (!imageReady) message = 'Waiting for map image to load…';
    else if (!connected) message = 'Waiting for ROS connection…';
    else if (state.localizationSubmitting) message = 'Publishing initial pose…';
    else if (unsentDraft) message = amclSubscriber ? 'Initial pose selected — click Localize to apply it' : 'Waiting for AMCL /initialpose subscriber…';
    else if (hasPose) message = 'Map pose available — click Continue if the robot triangle matches';
    else if (!nav) message = 'Waiting for navigation and a map-frame pose…';
    else if (state.initialPoseSent) message = 'Initial pose sent — waiting for a map-frame pose…';
    else message = 'Select a pose on the map, or wait for an existing map pose';
    return { ready, hasPose, canLocalize:ready && nav && !!state.pose && amclSubscriber,
             canContinue:ready && hasPose && !unsentDraft,
             message, error:!!state.localizationError };
}
function updateLocalizationControls() {
    if (state.mode !== 'localize') return;
    const g=localizationGate();
    const result=$('#locResult');
    if (result) {
        result.textContent=g.message;
        result.className=(g.error?'bad':g.canContinue?'good':'warntext');
    }
    const localize=$('#localizeStart');
    if (localize) localize.disabled=!g.canLocalize;
    const next=$('#localizeContinue');
    if (next) next.disabled=!g.canContinue;
    const retry=$('#localizationRetry');
    if (retry) retry.style.display=g.error?'inline-flex':'none';
    const reset=$('#localizeReset');
    if (reset) reset.disabled=!!(state.localizationLoading || state.localizationSubmitting);
}
async function loadMeta(name) {
    try {
        const result=await get('maps/'+name+'.meta');
        if (state.activeMap !== name || state.mode!=='localize') return false;
        state.mapMeta=result;
        paintMap();
        return true;
    } catch(e) { state.localizationError='Cannot load saved map: '+e.message; toast(state.localizationError,true); return false; }
}
async function sendInitialPose() {
    const g=localizationGate();
    if (!g.ready) return toast(g.message,true);
    if (!state.pose) return toast('Drag on the map to select a position and heading first',true);
    if (state.localizationSubmitting) return;
    state.localizationSubmitting=true;
    updateLocalizationControls();
    try {
        const r=await safe('initialpose',state.pose);
        if(r) { state.initialPoseSent=true; toast('Initial pose sent. Verify the robot arrow, then Continue.'); }
    } finally { state.localizationSubmitting=false; updateLocalizationControls(); }
}
function clearPose() {
    state.pose=null; state.initialPoseSent=false; state.localized=false;
    paintMap(); updateLocalizationControls();
}
function continueLocalization() {
    const g = localizationGate();
    if (!g.canContinue) return toast(g.message, true);
    // This is only a UI transition. Pose publishing is handled by Localize;
    // robot movement remains subject to ROS interlocks when requested.
    if (state.page === 'mission' && !state.mission)
        return toast('Mission data missing. Reopen your mission.', true);
    if (state.page === 'patrol' && !state.patrolMission)
        return toast('Patrol mission missing. Select a mission again.', true);
    const previousMode = state.mode;
    const previousLocalized = state.localized;
    state.localized = true;
    state.mode = state.page === 'patrol' ? 'live' : 'edit';
    try {
        render();
    } catch (error) {
        // Avoid silently leaving the old screen visible on a rendering error.
        state.mode = previousMode;
        state.localized = previousLocalized;
        toast('Could not open ' + (state.page === 'patrol' ? 'Patrol' : 'Mission Editor') +
              ': ' + (error && error.message ? error.message : error), true);
        try { render(); } catch (ignored) { console.error(ignored); }
        console.error('STIP localization Continue failed:', error);
        return;
    }
    state.pose = null;
    state.initialPoseSent = false;
    state.localizationError = null;
}
async function finishMap(save) { releaseTeleop(); await safe('teleop_arm', { enabled: false }); if (save) {
    const v = await safe('save_map', { name: state.activeMap });
    if (!v)
        return;
} await safe('stop_mapping'); await refreshLists(); state.mode = 'list'; page('maps', 'list'); }
async function toggleTeleop() {
    if (teleopChanging) return;
    const patrolManual=state.page==='patrol' && state.mode==='live';
    if (!((state.page === 'maps' && state.mode === 'mapping') || (state.page === 'mission' && state.mode === 'edit') || patrolManual)) return;
    teleopChanging = true;
    try {
        if (state.teleop) {
            // Disarming itself publishes a zero Twist in the ROS worker.
            // Never queue another teleop packet behind the disarm request.
            state.teleop=false;teleopSession=null;
            clearInterval(moveTimer); moveTimer=null;
            state.held=null; teleopStopPending=false;
            const r=await safe('teleop_arm',{enabled:false});
            if (!r) toast('Check teleop status: disarm was not acknowledged',true);
        } else {
            if(patrolManual && !confirm('Take manual control? This will STOP the patrol, CANCEL its navigation goal, and require restarting the mission from its FIRST waypoint.'))return;
            const r=await safe(patrolManual?'patrol_takeover':'teleop_arm',{enabled:true});
            if (r && r.armed) {
                teleopSeq=0;teleopSession=r.session;
                state.teleop=true;
                state.teleopStartAt=Date.now();
            }
        }
        render();
    } finally {teleopChanging=false;}
}
function releaseTeleop() {
    clearInterval(moveTimer);moveTimer=null;
    state.held=null;
    // Send one zero only if the UI still owns an armed teleop session.
    if (state.teleop) {teleopStopPending=true;flushTeleop();}
    else teleopStopPending=false;
}
async function flushTeleop() {
    if (teleopSending || !state.teleop || teleopChanging) return;
    teleopSending=true;
    try {
        const d=state.held;
        const linear=d==='w'?state.linear:d==='s'?-state.linear:0;
        const angular=d==='a'?state.angular:d==='d'?-state.angular:0;
        teleopStopPending=false;
        await act('teleop',{linear,angular,seq:++teleopSeq,session:teleopSession},2200);
    } catch(e) {
        // Never keep issuing commands after the ROS worker rejects control.
        const wasMoving=!!state.held;
        state.held=null;state.teleop=false;teleopSession=null;
        clearInterval(moveTimer);moveTimer=null;teleopStopPending=false;
        toast('Teleop disarmed: '+e.message,true);
        // Backend owns safety; explicitly request a zero-velocity disarm.
        act('teleop_arm',{enabled:false},2000).catch(()=>{});
        const button=document.querySelector('#fp_teleop button, #fp_missionteleop button, #fp_patrolteleop button');
        if (button) {button.textContent='Enable Teleop';button.classList.remove('red');button.classList.add('blue');}
    } finally {
        teleopSending=false;
        if (teleopStopPending && state.teleop) flushTeleop();
    }
}
function sendHeld() { if (state.teleop && state.held)
    flushTeleop(); }
function pressMove(d) { if (!state.teleop || document.querySelector('#modalBackdrop'))
    return; state.held = d; teleopStopPending = false; clearInterval(moveTimer); sendHeld(); moveTimer = setInterval(sendHeld, 90); }
function attachTeleopButtons() { $$('[data-move]').forEach(el => { el.addEventListener('pointerdown', e => { e.preventDefault(); el.setPointerCapture(e.pointerId); pressMove(el.dataset.move); }); el.addEventListener('pointerup', releaseTeleop); el.addEventListener('pointercancel', releaseTeleop); el.addEventListener('lostpointercapture', releaseTeleop); }); }
document.addEventListener('keydown', e => { if (/input|textarea|select/i.test(e.target.tagName) || e.repeat)
    return; let d = e.key.toLowerCase(); if ('wasd'.includes(d) && d.length === 1 && state.teleop && state.mode !== 'qr') {
    e.preventDefault();
    pressMove(d);
} });
document.addEventListener('keyup', e => { if (e.key.toLowerCase() === state.held)
    releaseTeleop(); });
function loseFocus() { stopQrArm(); releaseTeleop(); if (state.teleop) {
    state.teleop = false;teleopSession=null;
    safe('teleop_arm', { enabled: false });
} }
window.addEventListener('blur', loseFocus);
document.addEventListener('visibilitychange', () => { if (document.hidden)
    loseFocus(); });
window.addEventListener('pagehide', loseFocus);
// Saving a waypoint must NOT disarm a healthy manual session. A native
// confirm() can trigger window blur/visibilitychange, which previously issued
// teleop_arm(false). Use our non-blocking dashboard modal instead.
let waypointSaveBusy=false;
async function addWaypoint() {
    if (waypointSaveBusy || state.page!=='mission' || state.mode!=='edit') return;
    if (state.held) return toast('Release W/A/S/D and let the robot stop before saving a waypoint.',true);
    waypointSaveBusy=true;
    try {
        // A final acknowledged ZERO command ensures that no earlier velocity
        // packet can remain outstanding across pose capture.
        if (state.teleop) {
            clearInterval(moveTimer);moveTimer=null;teleopStopPending=false;
            while (teleopSending) await new Promise(resolve=>setTimeout(resolve,20));
            const zero=await act('teleop',{linear:0,angular:0,seq:++teleopSeq,session:teleopSession},2200);
            if (!zero || !zero.sent) throw Error('Teleop stop was not acknowledged; check control status');
        }
        if(state.page!=='mission' || state.mode!=='edit') return;
        const ros=state.status.ros||{};
        if (!ros.connected) throw Error('ROS adapter offline: check dashboard connection.');
        if (!ros.robot_map || ![ros.robot_map.x,ros.robot_map.y,ros.robot_map.yaw].every(Number.isFinite))
            throw Error('Live map pose unavailable: check localization and TF.');
        const robot=ros.robot_map;
        modal('Save Current Robot Pose?',
            `<p>Robot must be fully stationary.</p><p>X ${robot.x.toFixed(2)} m · Y ${robot.y.toFixed(2)} m · Heading ${(robot.yaw*180/Math.PI).toFixed(1)}°</p>`+
            `<p class="small muted">Manual Teleop remains armed; no need to restart the dashboard.</p>`,
            btn('Cancel','closeModal()')+btn('Save Pose','confirmWaypointPose()','green'));
    } catch(e) {toast('Cannot save waypoint: '+e.message,true);}
    finally {waypointSaveBusy=false;}
}
function confirmWaypointPose(){
    if (state.page!=='mission' || state.mode!=='edit') return closeModal();
    const ros=state.status.ros||{},robot=ros.robot_map;
    if (state.held || (ros.teleop && ros.teleop.moving))
        return toast('Robot moving: release manual drive before saving the waypoint.',true);
    if (!ros.connected || !robot || ![robot.x,robot.y,robot.yaw].every(Number.isFinite))
        return toast('Map-frame pose unavailable; check TF/localization.',true);
    const pts=state.mission.waypoints;let id='P'+(pts.length+1);
    while (pts.some(p=>p.id===id)) id='P'+(Number(id.slice(1))+1);
    pts.push({id,name:'Point '+(pts.length+1),x:robot.x,y:robot.y,yaw:robot.yaw,tasks:[]});
    state.pointIndex=pts.length-1;
    closeModal();render();paintMap();
}
function selectPoint(i) { state.pointIndex = i; render(); }
function updatePointName(name) { if (state.pointIndex >= 0)
    state.mission.waypoints[state.pointIndex].name = name; }
function deletePoint() { if (state.pointIndex < 0)
    return; if (!confirm('Delete this waypoint and its tasks?'))
    return; state.mission.waypoints.splice(state.pointIndex, 1); state.pointIndex = -1; render(); }
async function navigateSelected() { if(state.teleop)return toast('Release Teleop before sending a navigation goal',true); var _a, _b; const point = (_a = state.mission) === null || _a === void 0 ? void 0 : _a.waypoints[state.pointIndex]; if (!point)
    return; const status = state.status.processes || {}; if (!((_b = status.navigation) === null || _b === void 0 ? void 0 : _b.running)) {
    let started = await safe('start_navigation', { map: state.activeMap });
    if (!started)
        return;
} await safe('goal', point, () => toast('Navigation goal sent. Wait until arrival is confirmed.')); }
function addPause() { let p = state.mission.waypoints[state.pointIndex]; if (!p)
    return; const input = prompt('Pause duration (seconds)', '5'); if (input === null)
    return; let n = Number(input); if (!Number.isFinite(n) || n < 0 || n > 3600)
    return toast('Duration must be 0–3600 seconds', true); p.tasks.push({ type: 'pause', seconds: n }); render(); }
function moveTask(i, delta) { var _a; let a = ((_a = state.mission.waypoints[state.pointIndex]) === null || _a === void 0 ? void 0 : _a.tasks) || [], j = i + delta; if (j < 0 || j >= a.length)
    return; [a[i], a[j]] = [a[j], a[i]]; render(); }
function removeTask(i) { var _a; (_a = state.mission.waypoints[state.pointIndex]) === null || _a === void 0 ? void 0 : _a.tasks.splice(i, 1); render(); }
function openQrEditor() { if (state.pointIndex < 0)
    return; state.mode = 'qr'; render(); openQrDialog(); }
function closeQrEditor(goHome) { closeModal(); state.capture = null; state.roi = null; state.mode = 'edit'; if (goHome) { exitMissionAndLeave('home','list'); return; } render(); }
async function armNudge(axis, nudge) { let old = state.arm[axis], v = Math.min(axis === 'pan' ? 180 : 90, Math.max(0, old + nudge)); const r = await safe('arm', { axis, angle: v }); if (r) {
    state.arm[axis] = v;
    const el = $('#' + axis + 'Angle');
    if (el)
        el.textContent = v + '°';
} }
// One image request at a time; do not queue image loads or arm targets.
