// STIP dashboard module: map_view
function loadImageIntoMap() { const img = $('#mapImage'); if (!img)
    return; img.onload = () => { if (state.mode === 'localize') { state.mapImageReady=true; updateLocalizationControls(); } const ms = $('#mapStatus'); if (ms && state.mode === 'mapping')
    ms.textContent = 'Mapping · ' + state.activeMap; applyMapView(); }; img.onerror = () => { if (state.mode === 'localize') { state.mapImageReady=false; state.localizationError='Map image did not load. Retry localization setup.'; updateLocalizationControls(); } const el = $('#mapStatus'); if (el)
    el.textContent = state.mode === 'mapping' ? 'Waiting for the first SLAM map' : 'Saved map unavailable'; paintMap(); }; img.src = (state.page === 'maps' && state.mode === 'mapping' ? '/api/live_map' : '/api/maps/' + state.activeMap + '.png') + '?t=' + Date.now(); }
function applyMapView() { const im = $('#mapImage'); if (im)
    im.style.transform = `translate(${mapView.x}px,${mapView.y}px) rotate(${mapView.angle}rad) scale(${zoom})`; paintMap(); }
function resetMapView() { zoom = 1; mapView = { x: 0, y: 0, angle: 0 }; applyMapView(); }
function adjustZoom(f) { zoom = Math.max(.35, Math.min(8, zoom * f)); applyMapView(); }
function imageRect() { let img = $('#mapImage'), f = $('#mapframe'); if (!(img === null || img === void 0 ? void 0 : img.naturalWidth) || !f)
    return null; const w = f.clientWidth, h = f.clientHeight; const ratio = Math.min(w / img.naturalWidth, h / img.naturalHeight) * zoom; return { x: (w - img.naturalWidth * ratio) / 2, y: (h - img.naturalHeight * ratio) / 2, w: img.naturalWidth * ratio, h: img.naturalHeight * ratio }; }
function viewScreen(x, y, inverse = false) {
    const f = $('#mapframe');
    if (!f)
        return { x, y };
    let cx = f.clientWidth / 2, cy = f.clientHeight / 2;
    if (inverse) {
        x -= mapView.x;
        y -= mapView.y;
        const dx = x - cx, dy = y - cy, c = Math.cos(-mapView.angle), s = Math.sin(-mapView.angle);
        return { x: cx + dx * c - dy * s, y: cy + dx * s + dy * c };
    }
    const dx = x - cx, dy = y - cy, c = Math.cos(mapView.angle), s = Math.sin(mapView.angle);
    return { x: cx + dx * c - dy * s + mapView.x, y: cy + dx * s + dy * c + mapView.y };
}
function screenToWorld(ex, ey) { const f = $('#mapframe'), r = imageRect(), meta = state.mapMeta; if (!r || !meta)
    return null; const rect = f.getBoundingClientRect(), v = viewScreen(ex - rect.left, ey - rect.top, true); let u = (v.x - r.x) / r.w, t = (v.y - r.y) / r.h; if (u < 0 || u > 1 || t < 0 || t > 1)
    return null; const px = u * meta.width * meta.resolution, py = (1 - t) * meta.height * meta.resolution, a = meta.origin_yaw || 0; return { x: meta.origin[0] + px * Math.cos(a) - py * Math.sin(a), y: meta.origin[1] + px * Math.sin(a) + py * Math.cos(a) }; }
function worldToScreen(x, y) { const r = imageRect(), meta = state.mapMeta; if (!r || !meta)
    return null; let dx = x - meta.origin[0], dy = y - meta.origin[1], a = -(meta.origin_yaw || 0); const px = dx * Math.cos(a) - dy * Math.sin(a), py = dx * Math.sin(a) + dy * Math.cos(a); return viewScreen(r.x + px / (meta.width * meta.resolution) * r.w, r.y + (1 - py / (meta.height * meta.resolution)) * r.h); }
// Both live robot and localization draft use the same map-frame heading.
// A triangle has a single unambiguous nose; no circle-and-line overlay.
function drawPoseTriangle(ctx, pose, length, fill) {
    if (!pose || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) return;
    const p = worldToScreen(pose.x,pose.y);
    const forward = worldToScreen(pose.x + .25*Math.cos(pose.yaw),
                                  pose.y + .25*Math.sin(pose.yaw));
    if (!p || !forward) return;
    let dx=forward.x-p.x, dy=forward.y-p.y;
    const mag=Math.hypot(dx,dy);
    if (!(mag>0.0001)) return;
    dx/=mag;dy/=mag;
    const sideX=-dy,sideY=dx;
    // Pointed nose with a flat rear makes the forward direction obvious.
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(p.x+dx*length*.8,p.y+dy*length*.8);
    ctx.lineTo(p.x-dx*length*.65+sideX*length*.47,p.y-dy*length*.65+sideY*length*.47);
    ctx.lineTo(p.x-dx*length*.65-sideX*length*.47,p.y-dy*length*.65-sideY*length*.47);
    ctx.closePath();
    ctx.fillStyle=fill;ctx.fill();
    ctx.lineWidth=2;ctx.strokeStyle='#f3fbff';ctx.stroke();
    ctx.restore();
}
function paintMap() {
    var _a, _b, _c, _d, _e;
    let c = $('#mapCanvas');
    if (!c)
        return;
    const f = $('#mapframe'), ratio = window.devicePixelRatio || 1, w = f.clientWidth, h = f.clientHeight;
    if (!w || !h)
        return;
    c.width = Math.round(w * ratio);
    c.height = Math.round(h * ratio);
    const ctx = c.getContext('2d');
    ctx.scale(ratio, ratio);
    const rstate = state.status.ros || {}, mapMode = state.page === 'maps' && state.mode === 'mapping';
    // Live laser scan is already projected into the map frame by TF on Jetson.
    if (mapMode || state.page === 'patrol' || state.page === 'mission') {
        const laser = ((_a = rstate.health) === null || _a === void 0 ? void 0 : _a.lidar) ? (((_b = rstate.laser) === null || _b === void 0 ? void 0 : _b.points) || []) : [];
        ctx.fillStyle = 'rgba(25,228,242,.85)';
        for (const xy of laser) {
            let q = worldToScreen(xy[0], xy[1]);
            if (q) {
                ctx.fillRect(q.x - 1.2, q.y - 1.2, 2.4, 2.4);
            }
        }
        const robot = rstate.robot_map;
        if (robot) drawPoseTriangle(ctx, robot, 17, '#238bff');
    }
    const pts = state.page === 'mission' ? ((_c = state.mission) === null || _c === void 0 ? void 0 : _c.waypoints) || [] : state.page === 'patrol' ? ((_d = state.patrolMission) === null || _d === void 0 ? void 0 : _d.waypoints) || [] : [];
    ctx.save();
    ctx.strokeStyle = '#4db0ff';
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    ctx.beginPath();
    pts.forEach((p, i) => { let q = worldToScreen(p.x, p.y); if (q) {
        if (!i)
            ctx.moveTo(q.x, q.y);
        else
            ctx.lineTo(q.x, q.y);
    } });
    if (state.page === 'patrol' && ((_e = state.patrolMission) === null || _e === void 0 ? void 0 : _e.loop) && pts.length) {
        let q = worldToScreen(pts[0].x, pts[0].y);
        if (q)
            ctx.lineTo(q.x, q.y);
    }
    ctx.stroke();
    ctx.restore();
    pts.forEach((p, i) => { const q = worldToScreen(p.x, p.y); if (!q)
        return; drawPoseTriangle(ctx, { x: p.x, y: p.y, yaw: Number.isFinite(p.yaw) ? p.yaw : 0 }, 15, i === state.pointIndex ? '#217dff' : '#126c9a'); ctx.fillStyle = 'white'; ctx.font = 'bold 10px sans-serif'; ctx.textAlign = 'center'; ctx.fillText('' + (i + 1), q.x, q.y + 4); });
    if (state.pose) drawPoseTriangle(ctx, state.pose, 20, '#25d99e');

}
let mapPointer = null;
function attachMapInteractions() {
    const c = $('#mapCanvas');
    if (!c)
        return;
    c.addEventListener('contextmenu', e => e.preventDefault());
    c.addEventListener('wheel', e => { e.preventDefault(); adjustZoom(e.deltaY < 0 ? 1.12 : 1 / 1.12); }, { passive: false });
    c.addEventListener('pointerdown', e => {
        if (e.button === 2) {
            e.preventDefault();
            mapPointer = { kind: 'pan', x: e.clientX, y: e.clientY };
            c.setPointerCapture(e.pointerId);
            return;
        }
        if (e.button !== 0)
            return;
        if (state.page === 'maps' && state.mode === 'mapping') {
            mapPointer = { kind: 'rotate', x: e.clientX, angle: mapView.angle };
            c.setPointerCapture(e.pointerId);
            return;
        }
        if (state.mode === 'localize') {
            if (!localizationGate().ready) return;
            const pt = screenToWorld(e.clientX, e.clientY);
            if (pt) {
                state.pose = { ...pt, yaw: 0 };
                state.initialPoseSent=false;
                updateLocalizationControls();
                mapPointer = { kind: 'pose', x: e.clientX, y: e.clientY, start: pt };
                c.setPointerCapture(e.pointerId);
                paintMap();
            }
        }
        else if (state.page === 'mission' && state.mode === 'edit') { /* Mission waypoints now use the robot's current map-frame pose. */ }
    });
    c.addEventListener('pointermove', e => {
        if (!mapPointer)
            return;
        if (mapPointer.kind === 'pan') {
            mapView.x += e.clientX - mapPointer.x;
            mapView.y += e.clientY - mapPointer.y;
            mapPointer.x = e.clientX;
            mapPointer.y = e.clientY;
            applyMapView();
        }
        else if (mapPointer.kind === 'rotate') {
            mapView.angle = mapPointer.angle + (e.clientX - mapPointer.x) * .006;
            applyMapView();
        }
        else if (mapPointer.kind === 'pose' && state.pose) {
            let d = mapPointer;
            const target = screenToWorld(e.clientX, e.clientY);
            if (target && Math.hypot(target.x - d.start.x, target.y - d.start.y) > 0.03) {
                state.pose.yaw = Math.atan2(target.y - d.start.y, target.x - d.start.x);
            }
            paintMap();
        }
    });
    c.addEventListener('pointerup', () => mapPointer = null);
    c.addEventListener('pointercancel', () => mapPointer = null);
}
