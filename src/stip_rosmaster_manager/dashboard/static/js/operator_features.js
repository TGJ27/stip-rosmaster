// Operator features: profile inspection/test; one-shot confirmed fire evidence;
// ROS-native detector settings and fire controller auto-aim.
let qrTaskToInspect=null;
let qrTestRunning=false;
let qrDialogVersion=0;
function qrProfilePhoto(name,kind) {
    return '/api/qr_profiles/'+encodeURIComponent(name)+'/'+kind+'.jpg';
}
function qrReferencePanel(name, kind, title, description) {
    return '<figure class="operator-qr-picture">'+
        '<div class="operator-qr-frame"><img src="'+qrProfilePhoto(name,kind)+'" alt="'+esc(description)+'" loading="lazy"></div>'+
        '<figcaption><strong>'+esc(title)+'</strong><span>'+esc(description)+'</span></figcaption></figure>';
}
function selectedQrTask(index) {
    const point=state.mission && state.mission.waypoints[state.pointIndex];
    const task=point && point.tasks[index];
    return task && task.type==='qr' ? task : null;
}
// Profile viewer and inspection execution are deliberately separate screens.
function viewQrTask(index) { return openQrTaskDialog(index,false); }
function openQrTestDialog(index) { return openQrTaskDialog(index,true); }
async function openQrTaskDialog(index, isTest) {
    const task=selectedQrTask(index);
    if(!task)return;
    const profileName=task.profile;
    const version=++qrDialogVersion;
    qrTaskToInspect=isTest?profileName:null;
    const title=isTest?'QR Inspection Test':'Saved QR Profile';
    modal(title,'<p class="small muted">Loading saved QR reference…</p>');
    const backdrop=$('#modalBackdrop');
    if(backdrop)backdrop.classList.add('saved-qr-full');
    try {
        const data=await get('qr_profiles/'+encodeURIComponent(profileName));
        // A closed or replaced dialog must never be resurrected by a slow response.
        if(version!==qrDialogVersion || $('#modalBackdrop')!==backdrop)return;
        const profile=data.profile;
        const qr=profile.qr || {}, indicator=profile.indicator || {}, pose=profile.camera_reference || {};
        const fmtAngle=value=>Number.isFinite(Number(value))?Number(value).toFixed(1)+'°':'—';
        let html='<p class="small muted">ROS inspection profile: <b>'+esc(profile.name)+'</b></p>'+
            '<div class="qr-profile-meta"><div><span>QR value</span><b>'+esc(qr.value || '—')+'</b></div>'+
            '<div><span>Pan / tilt</span><b>'+fmtAngle(pose.pan)+' / '+fmtAngle(pose.tilt)+'</b></div>'+
            '<div><span>Indicator ROI</span><b>'+esc(JSON.stringify(indicator.bbox || {}))+'</b></div></div>'+
            '<div class="operator-qr-gallery'+(isTest?'':' profile-only')+'">';
        if(isTest)html+='<figure class="operator-qr-picture"><div class="operator-qr-frame"><img id="qrSavedLive" src="/api/camera.mjpg" alt="Live inspection camera"></div><figcaption><strong>Live camera</strong><span>Updates during the inspection test</span></figcaption></figure>';
        if(profile.images && profile.images.annotated)html+=qrReferencePanel(profile.name,'annotated','Enrolled QR & indicator','Saved detection boxes');
        if(profile.images && profile.images.reference)html+=qrReferencePanel(profile.name,'reference','Original frame','Saved enrollment image');
        html+='</div>';
        if(isTest){
            html+='<div id="qrTestResult" class="small muted" role="status" aria-live="polite">'+
                (qrTestRunning?'A QR test is already running. Wait for the result.':'The robot must be stopped. The test calls /inspection/run at the current position; it never navigates to this waypoint.')+
                '</div><div class="flexwrap operator-qr-actions"><button class="btn blue" id="qrTestStart" onclick="testQrTask()" '+(qrTestRunning?'disabled':'')+'>Run Inspection Test</button>'+
                '<button class="btn" onclick="closeQrTaskDialog()">Close</button></div>';
        }else{
            html+='<div class="flexwrap operator-qr-actions"><button class="btn" onclick="closeQrTaskDialog()">Close Profile</button></div>';
        }
        // Modal replacement keeps the task dialog independent of the evidence gallery.
        modal(title,html);
        $('#modalBackdrop').classList.add('saved-qr-full');
    }catch(e){
        if(version!==qrDialogVersion || $('#modalBackdrop')!==backdrop)return;
        modal('QR Profile Unavailable','<p class="bad">'+esc(e.message)+'</p><p class="small muted">Removing a task does not delete the ROS profile on disk.</p><button class="btn" onclick="closeQrTaskDialog()">Close</button>');
    }
}
function closeQrTaskDialog(){
    ++qrDialogVersion;
    qrTaskToInspect=null;
    closeModal();
}
async function testQrTask() {
    if(!qrTaskToInspect || qrTestRunning)return;
    const profile=qrTaskToInspect;
    const version=qrDialogVersion;
    const area=$('#qrTestResult'),button=$('#qrTestStart');
    const backdrop=$('#modalBackdrop');
    qrTestRunning=true;
    if(button){button.disabled=true;button.textContent='Testing…';}
    if(area)area.textContent='Testing saved profile via /inspection/run. Live camera stays on. No navigation goal is sent.';
    try {
        const result=await act('qr_test',{profile:profile},90000);
        if(version!==qrDialogVersion || $('#modalBackdrop')!==backdrop)return;
        const current=$('#qrTestResult');
        if(current)current.innerHTML='<b class="'+(result.success?'good':'bad')+'">'+(result.success?'PASS':'FAIL')+'</b> — '+esc(result.message||'')+'<p class="small muted">Inspection evidence is saved by the ROS package and can be opened separately through Inspection Gallery.</p>';
    }catch(e){
        if(version!==qrDialogVersion || $('#modalBackdrop')!==backdrop)return;
        const current=$('#qrTestResult');
        if(current)current.textContent='Inspection test: '+e.message;
        toast('QR test failed: '+e.message,true);
    }finally{
        qrTestRunning=false;
        const currentButton=$('#qrTestStart');
        if(currentButton && version===qrDialogVersion && $('#modalBackdrop')===backdrop){currentButton.disabled=false;currentButton.textContent='Run Inspection Test';}
    }
}
async function loadFireSettings(){
    const hidden=$('#fireSettingsLoaded');if(!hidden || hidden.value)return;
    hidden.value='loading';
    try {
        const r=await get('fire_thresholds');
        if($('#fireConfInput'))$('#fireConfInput').value=r.values.fire_confidence_threshold.toFixed(2);
        if($('#smokeConfInput'))$('#smokeConfInput').value=r.values.smoke_confidence_threshold.toFixed(2);
        if($('#fireSettingsStatus'))$('#fireSettingsStatus').textContent='Settings from native C++ ROS fire detector configuration. Apply requires restart.';
        hidden.value='ready';
    }catch(e){hidden.value='error';if($('#fireSettingsStatus'))$('#fireSettingsStatus').textContent=e.message;}
}
async function saveFireSettings(){
    const fire=Number($('#fireConfInput').value),smoke=Number($('#smokeConfInput').value);
    if(!Number.isFinite(fire)||!Number.isFinite(smoke)||fire<0.1||smoke<0.1||fire>0.95||smoke>0.95)return toast('Confidence must be between 0.10 and 0.95',true);
    if(!confirm('Save new fire/smoke detector thresholds and restart ONLY the managed fire detection stage? Stop Patrol first. This may briefly interrupt monitoring.'))return;
    const result=await safe('fire_thresholds_set',{fire,smoke,ack_restart:true});
    if(result){toast(result.restart_queued?'Saved. Fire stage restart queued; await READY before relying on monitoring.':'Thresholds unchanged.');}
}
async function toggleFireTracking(enabled){
    if(!confirm((enabled?'Enable':'Disable')+' original ROS fire-inspection auto-aim? Robot must be stationary and the fire controller must own the arm.'))return;
    const r=await safe('fire_tracking',{enabled});if(r)toast(r.message||'Fire tracking updated');
}
async function openFireGallery(){
    modal('Confirmed Fire Evidence','<p class="small muted">Loading saved ROS confirmation snapshots…</p>');
    try{
        const result=await get('fire_events');
        const events=result.events||[];
        const html='<p class="small muted">One snapshot per new ROS-confirmed fire alarm, newest first. Captures RGB, thermal preview (when available) and fusion data.</p><div class="operator-gallery">'+
            events.map((x,i)=>'<button class="operator-entry" onclick="openFireEvent(\''+esc(x.event_id)+'\')"><b>'+esc(x.timestamp||x.event_id)+'</b><div>Fire CONFIRMED · Target: '+esc(x.hot_c===null?'—':x.hot_c)+' °C · Background: '+esc(x.background_c===null?'—':x.background_c)+' °C</div></button>').join('')+'</div>';
        modal('Confirmed Fire Evidence ('+events.length+')',html);
    }catch(e){modal('Fire Evidence','<p class="bad">'+esc(e.message)+'</p>');}
}
async function openFireEvent(eventId){
    if(!/^fire_[0-9]{8}_[0-9]{6}_[0-9a-f]{8}$/.test(eventId))return;
    const result=await get('fire_events');
    const event=(result.events||[]).find(x=>x.event_id===eventId);
    if(!event)return;
    const images=event.images||{};
    const base='/api/fire_events/'+encodeURIComponent(eventId)+'/';
    let html='<p>Confirmed at '+esc(event.timestamp)+'</p><p class="small muted">Fusion state: '+esc(event.fusion_state)+'; target '+esc(event.hot_c)+' °C; background '+esc(event.background_c)+' °C.</p><div class="operator-images">';
    if(images.rgb)html+='<figure><img src="'+base+'rgb.jpg"><figcaption>RGB at confirmation</figcaption></figure>';
    if(images.thermal)html+='<figure><img src="'+base+'thermal.jpg"><figcaption>Thermal preview at confirmation (not temperature measurement)</figcaption></figure>';
    html+='</div><pre style="white-space:pre-wrap;max-height:180px;overflow:auto">'+esc(JSON.stringify(event.fusion||{},null,2))+'</pre><button class="btn" onclick="openFireGallery()">Back</button>';
    modal('Fire Confirmation',html);
}

// Existing fire-inspection ROS timing parameters, loaded from native inspection.yaml.
async function loadTrackingSpeed(){
    const hidden=$('#trackingSettingsLoaded');if(!hidden || hidden.value)return;
    hidden.value='loading';
    try{
        const result=await get('tracking_speed');
        if($('#trackingStepInput'))$('#trackingStepInput').value=result.values.max_servo_step_deg.toFixed(2);
        if($('#trackingIntervalInput'))$('#trackingIntervalInput').value=result.values.min_servo_interval_sec.toFixed(2);
        if($('#trackingSettingsStatus'))$('#trackingSettingsStatus').textContent='Loaded from fire-inspection ROS configuration. Restart required after changing.';
        hidden.value='ready';
    }catch(e){hidden.value='error';if($('#trackingSettingsStatus'))$('#trackingSettingsStatus').textContent=e.message;}
}
async function saveTrackingSpeed(){
    const step=Number($('#trackingStepInput').value), interval=Number($('#trackingIntervalInput').value);
    if(!Number.isFinite(step)||!Number.isFinite(interval)||step<0.5||step>3||interval<0.18||interval>1)
        return toast('Step: 0.5–3 degrees; interval: 0.18–1.0 seconds',true);
    if(!confirm('Save fire-inspection tracking speed? Stop Patrol and turn Fire Monitoring OFF first. Changes require a controller restart.'))return;
    const result=await safe('tracking_speed_set',{step,interval,ack_restart:true});
    if(result){
        const status=$('#trackingSettingsStatus');
        if(status)status.textContent=result.message||(result.changed?'Saved. Restart required.':'No change.');
        toast(result.message||(result.changed?'Tracking speed saved':'Tracking speed unchanged'));
    }
}
