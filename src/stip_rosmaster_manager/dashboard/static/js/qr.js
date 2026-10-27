// STIP dashboard module: qr
function stopQrCamera() {
    qrCamActive = false;
    qrCamGeneration++;
    if (qrLiveTimer) {
        clearTimeout(qrLiveTimer);
        qrLiveTimer = null;
    }
    const preview = $('#captureImg');
    if (preview)
        preview.removeAttribute('src');
    if (qrProbeTimer) {
        clearInterval(qrProbeTimer);
        qrProbeTimer = null;
    }
    qrCamBusy = false;
}
function refreshQrCamera(generation) {
    const img = $('#captureImg');
    if (img && qrCamActive && generation === qrCamGeneration)
        img.src = '/api/camera.mjpg';
}
function startQrCamera() {
    stopQrCamera();
    qrCamActive = true;
    const generation = qrCamGeneration;
    refreshQrCamera(generation);
    // QR decoding runs only when Enroll QR is clicked; keep live preview lightweight.
}
function openQrDialog() {
    var _a, _b;
    const pt = (_b = (_a = state.mission) === null || _a === void 0 ? void 0 : _a.waypoints) === null || _b === void 0 ? void 0 : _b[state.pointIndex];
    if (!pt)
        return;
    const name = 'QR_' + pt.id;
    state.capture = null;
    state.roi = null;
    modal('QR Enrollment', `<div class="qr-layout"><div class="qr-stage"><div class="qr-image" id="captureFrame"><img id="captureImg" alt="Live RGB camera" src="/api/camera.mjpg" draggable="false"><svg class="qr-detected-overlay" id="qrDetectedBox" preserveAspectRatio="xMidYMid meet" aria-label="Detected QR outline" style="display:none"></svg><div class="roi" id="roiBox" style="display:none"></div></div><div class="qr-help" style="padding:9px" id="qrStatus">Live camera. Aim with W/A/S/D, then click Enroll QR. If a QR is readable, draw the indicator box.</div></div><div class="qr-side"><label class="field">Profile name<input id="profileName" value="${esc(name)}" maxlength="64"></label><p class="small muted">Changes are saved only when you click Save Enrollment. A previous version is backed up automatically if the name already exists.</p><div class="metric"><span>Pan</span><b id="panAngle">${state.arm.pan.toFixed(0)}°</b></div><div class="metric"><span>Tilt</span><b id="tiltAngle">${state.arm.tilt.toFixed(0)}°</b></div><div class="qr-pad"><span></span><button class="btn" data-qrarm="w">W ↑</button><span></span><button class="btn" data-qrarm="a">A ←</button><button class="btn" data-qrarm="s">S ↓</button><button class="btn" data-qrarm="d">D →</button></div><div class="qr-help">W/S: tilt · A/D: pan. Keyboard works while this window is open and no text field is focused. Hold to move; release to stop. Small repeated commands.</div><button class="btn green" id="qrEnrollButton" onclick="beginQrEnroll()">Enroll QR</button><button class="btn" id="qrRetryButton" onclick="resetQrPreview()" style="display:none">Back to Live</button><button class="btn" onclick="closeQrEditor()">Cancel</button></div></div>`);
    $('#modalBackdrop').classList.add('qr-full');
    attachQrArmButtons();
    const backdrop = $('#modalBackdrop');
    backdrop.onclick = e => { if (e.target === backdrop)
        closeQrEditor(); };
    startQrCamera();
}
// Browser communicates control state; ROS worker owns stepping and a deadman timeout.
let qrArmSeq = 0, qrArmPulseTimer = null;
function stopQrArm() {
    const wasHeld = qrHeld;
    qrHeld = null;
    if (qrArmPulseTimer !== null) {
        clearInterval(qrArmPulseTimer);
        qrArmPulseTimer = null;
    }
    if (qrArmLoop !== null) {
        clearInterval(qrArmLoop);
        qrArmLoop = null;
    }
    // No repeat commands after release. Sequence defeats late arrival/reordering.
    if (wasHeld)
        act('arm_hold', { op: 'stop', seq: ++qrArmSeq }).catch(e => toast('Arm release failed: ' + e.message, true));
}
function startQrArm(key) {
    if (state.mode !== 'qr' || state.capture || !['w', 'a', 's', 'd'].includes(key))
        return;
    if (qrHeld === key)
        return;
    stopQrArm();
    qrHeld = key;
    const pulse = () => {
        if (qrHeld !== key)
            return;
        act('arm_hold', { op: 'pulse', key, seq: ++qrArmSeq }).catch(e => { stopQrArm(); toast('Arm control: ' + e.message, true); });
    };
    pulse();
    qrArmPulseTimer = setInterval(pulse, 150);
}
function attachQrArmButtons() {
    document.querySelectorAll('[data-qrarm]').forEach(el => {
        el.addEventListener('pointerdown', e => { e.preventDefault(); el.setPointerCapture(e.pointerId); startQrArm(el.dataset.qrarm); });
        el.addEventListener('pointerup', stopQrArm);
        el.addEventListener('pointerleave', stopQrArm);
        el.addEventListener('pointercancel', stopQrArm);
        el.addEventListener('lostpointercapture', stopQrArm);
    });
}
let qrEnrollPending=false;
async function beginQrEnroll() {
    if (qrEnrollPending) return;
    if (state.capture) return enrollQr();
    stopQrArm();
    qrEnrollPending=true;
    const button=$('#qrEnrollButton');
    if (button) {button.disabled=true;button.textContent='Checking QR…';}
    let r;
    try {r=await safe('qr_prepare',{});}
    finally {
        qrEnrollPending=false;
        if (!r && button) {button.disabled=false;button.textContent='Enroll QR';}
    }
    if (!r) return;
    if (!r.detected) {
        if (button) {button.disabled=false;button.textContent='Enroll QR';}
        toast(r.detail || 'No QR detected', true);
        return;
    }
    if (state.mode!=='qr') return;
    stopQrCamera();
    state.capture = r;
    state.roi = null;
    const img = $('#captureImg');
    if (img) {
        img.onload = () => { if (state.capture === r) drawQrDetectedBox(r.qr_points || null); };
        img.src = r.jpeg;
    }
    const frame = $('#captureFrame');
    if (frame)
        frame.classList.add('roi-select');
    const el = $('#qrStatus');
    if (el)
        el.textContent = 'QR detected. Drag directly on the image to mark the indicator box.';
    const btn = $('#qrEnrollButton');
    if (btn) {
        btn.disabled = true;
        btn.textContent = 'Save Enrollment';
    }
    drawQrDetectedBox(r.qr_points || null);
    const retry = $('#qrRetryButton');
    if (retry)
        retry.style.display = 'block';
    const box = $('#roiBox');
    if (box)
        box.style.display = 'none';
    attachRoi();
}

function drawQrDetectedBox(points) {
    const box = $('#qrDetectedBox'), img = $('#captureImg');
    if (!box || !img || !Array.isArray(points) || points.length < 4 || !img.naturalWidth || !img.naturalHeight) {
        if (box) box.style.display = 'none';
        return;
    }
    // SVG viewBox maps ROS pixel coordinates through the same contain fit as
    // the image. Resize and letterboxing can no longer offset the polygon.
    const w = img.naturalWidth, h = img.naturalHeight;
    if (!points.every(p => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite))) return;
    box.setAttribute('viewBox', `0 0 ${w} ${h}`);
    const path=points.map(p=>p[0]+','+p[1]).join(' ');
    const sw=Math.max(2,Math.round(w/260));
    box.innerHTML=`<polygon points="${path}" fill="rgba(35,220,103,.08)" stroke="#28eb68" stroke-width="${sw}" stroke-linejoin="round"/>`;
    box.style.display = 'block';
}

function resetQrPreview() {
    stopQrArm();
    if (state.mode !== 'qr')
        return;
    state.capture = null;
    state.roi = null;
    const frame = $('#captureFrame');
    if (frame)
        frame.classList.remove('roi-select');
    const box = $('#roiBox');
    if (box)
        box.style.display = 'none';
    const qbox = $('#qrDetectedBox');
    if (qbox)
        qbox.style.display = 'none';
    const button = $('#qrEnrollButton');
    if (button) {
        button.textContent = 'Enroll QR';
        button.disabled = false;
    }
    const retry = $('#qrRetryButton');
    if (retry)
        retry.style.display = 'none';
    const el = $('#qrStatus');
    if (el)
        el.textContent = 'Live camera — aim at QR then click Enroll QR.';
    startQrCamera();
}
function attachRoi() {
    const img = $('#captureImg'), frame = $('#captureFrame');
    if (!img || !frame || !state.capture)
        return;
    let anchor = null;
    function bounds() {
        const r = img.getBoundingClientRect();
        const nw = img.naturalWidth || 1, nh = img.naturalHeight || 1;
        const scale = Math.min(r.width / nw, r.height / nh), w = nw * scale, h = nh * scale;
        return { left: r.left + (r.width - w) / 2, top: r.top + (r.height - h) / 2, width: w, height: h };
    }
    const clamp = v => Math.max(0, Math.min(1, v));
    // Handlers are replaced, never stacked across repeated enrollment attempts.
    frame.onpointerdown = e => {
        if (e.button !== 0 || !state.capture)
            return;
        const r = bounds();
        if (e.clientX < r.left || e.clientX > r.left + r.width || e.clientY < r.top || e.clientY > r.top + r.height)
            return;
        e.preventDefault();
        e.stopPropagation();
        anchor = { x: clamp((e.clientX - r.left) / r.width), y: clamp((e.clientY - r.top) / r.height) };
        frame.setPointerCapture(e.pointerId);
        state.roi = null;
    };
    frame.onpointermove = e => {
        if (!anchor)
            return;
        e.preventDefault();
        e.stopPropagation();
        const r = bounds(), x = clamp((e.clientX - r.left) / r.width), y = clamp((e.clientY - r.top) / r.height);
        state.roi = { x: Math.min(x, anchor.x), y: Math.min(y, anchor.y), w: Math.abs(x - anchor.x), h: Math.abs(y - anchor.y) };
        const rect = $('#roiBox');
        if (!rect)
            return;
        const fr = frame.getBoundingClientRect();
        rect.style.display = 'block';
        rect.style.left = (r.left - fr.left + state.roi.x * r.width) + 'px';
        rect.style.top = (r.top - fr.top + state.roi.y * r.height) + 'px';
        rect.style.width = (state.roi.w * r.width) + 'px';
        rect.style.height = (state.roi.h * r.height) + 'px';
    };
    frame.onpointerup = e => {
        if (!anchor)
            return;
        e.preventDefault();
        e.stopPropagation();
        anchor = null;
        if (state.roi && state.roi.w > 0.01 && state.roi.h > 0.01) {
            const button = $('#qrEnrollButton');
            if (button)
                button.disabled = false;
            const el = $('#qrStatus');
            if (el)
                el.textContent = 'Indicator selected — click Save Enrollment.';
        }
        else {
            state.roi = null;
            const box = $('#roiBox');
            if (box)
                box.style.display = 'none';
        }
    };
    frame.onpointercancel = () => { anchor = null; };
}
let qrSavePending = false;
async function enrollQr() {
    var _a;
    if (!state.capture || !state.roi)
        return toast('Draw the indicator rectangle first', true);
    const name = (_a = $('#profileName')) === null || _a === void 0 ? void 0 : _a.value.trim();
    // Enrollment is a commit operation, not an intermediate UI step. Existing
    // profiles with the same name are backed up and replaced on Save only.
    if (qrSavePending) return;
    qrSavePending = true;
    const saveButton = $('#qrEnrollButton');
    if (saveButton) { saveButton.disabled = true; saveButton.textContent = 'Saving…'; }
    try {
    const r = await safe('qr_enroll', { name, frame_id: state.capture.frame_id, roi: state.roi, replace_existing:true });
    if (r) {
        const tasks = state.mission.waypoints[state.pointIndex].tasks;
        const replacement = { type: 'qr', profile: r.profile, qr_value: r.qr_value, pan: r.pan, tilt: r.tilt };
        const existing = tasks.findIndex(t => t.type === 'qr' && t.profile === r.profile);
        if (existing >= 0) tasks[existing] = replacement;
        else tasks.push(replacement);
        // Return Home only after persisting the mission referencing this QR.
        // Otherwise the profile exists on disk but the waypoint task vanishes
        // when the operator later opens the mission again.
        const saved = await safe('save_mission', { mission: state.mission });
        if (saved) {
            state.mission = saved;
            await refreshLists();
            toast('QR enrolled and mission saved');
            closeQrEditor(true);
        } else {
            toast('QR profile saved, but mission save failed. Save Mission before leaving.', true);
            closeQrEditor(false);
        }
    }
    } finally {
        qrSavePending = false;
        const button = $('#qrEnrollButton');
        if (button && state.mode === 'qr' && state.capture) {
            button.disabled = !state.roi;
            button.textContent = 'Save Enrollment';
        }
    }
}
