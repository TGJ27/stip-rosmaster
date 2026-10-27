// Inspection gallery: read-only snapshots saved by the original stip_qr_inspection package.
let inspectionGalleryRuns = [];
let inspectionGalleryRequest = 0;
function closeInspectionGallery(){
    // Invalidate any outstanding history request before closing the dialog.
    inspectionGalleryRequest++;
    closeModal();
}
function inspectionGalleryDialog(title, body){
    modal(title, '<button type="button" class="gallery-close" aria-label="Close inspection gallery" title="Close gallery" onclick="closeInspectionGallery()">×</button>'+body);
    const backdrop=$('#modalBackdrop');
    if(backdrop)backdrop.classList.add('gallery-full');
}

function inspectionAsset(row, kind) {
    return '/api/inspection_runs/' + encodeURIComponent(row.profile) + '/' +
           encodeURIComponent(row.run) + '/' + kind + '.jpg';
}
async function openInspectionGallery() {
    const request=++inspectionGalleryRequest;
    inspectionGalleryDialog('Inspection Gallery', '<div class="small muted">Loading saved inspection results…</div>');
    const backdrop=$('#modalBackdrop');
    try {
        const response=await get('inspection_runs');
        // Closing or replacing the gallery must not reopen it after a late response.
        if(request!==inspectionGalleryRequest || $('#modalBackdrop')!==backdrop)return;
        inspectionGalleryRuns=response.runs || [];
        showInspectionGalleryList();
    }catch(error){
        if(request!==inspectionGalleryRequest || $('#modalBackdrop')!==backdrop)return;
        inspectionGalleryDialog('Inspection Gallery', '<p class="bad">'+esc(error.message)+'</p>');
    }
}
function showInspectionGalleryList() {
    const list = inspectionGalleryRuns;
    const markup = '<p class="small muted">Newest first · saved by the ROS QR inspection package · click a card to view full evidence.</p>' +
        '<div class="gallery-results">' +
        (list.map((run, i) => {
            const img = run.images.indicator ? inspectionAsset(run, 'indicator') :
                        run.images.annotated ? inspectionAsset(run, 'annotated') : '';
            return '<button class="inspection-gallery-card" type="button" onclick="showInspectionGalleryRun(' + i + ')">' +
                (img ? '<img loading="lazy" src="' + img + '" alt="Inspection evidence">' : '<span class="no-image">No image</span>') +
                '<span class="gallery-card-meta"><b>' + esc(run.inspection_name) + '</b><span>' + esc(run.timestamp || run.run) + '</span>' +
                '<span class="' + (run.aligned ? 'good' : 'warntext') + '">' + (run.aligned ? 'ALIGNED' : 'NOT ALIGNED') +
                '</span><span>' + esc(run.message) + '</span></span></button>';
        }).join('') || '<div class="small muted">No saved inspections yet. Run a QR inspection during patrol.</div>') +
        '</div>';
    inspectionGalleryDialog('Inspection Gallery (' + list.length + ')', markup);
}
function showInspectionGalleryRun(index) {
    const run = inspectionGalleryRuns[index];
    if (!run) return;
    const images = [['annotated', 'Detected QR / indicator'], ['indicator', 'Inspected indicator crop'], ['original', 'Original camera frame']]
        .filter(([kind]) => run.images[kind]);
    const body = '<div class="gallery-detail"><p class="small muted">' + esc(run.timestamp || run.run) +
        ' · ' + esc(run.inspection_name) + ' · ' + (run.aligned ? 'ALIGNED' : 'NOT ALIGNED') + '</p>' +
        '<p>' + esc(run.message) + '</p>' +
        '<p class="small muted">Indicator analysis: ' + esc(run.indicator_status) +
        ' (a QR alignment PASS is not a visual indicator quality verdict)</p>' +
        images.map(([kind,label]) => '<section class="gallery-image-section"><b>' + esc(label) + '</b><img src="' + inspectionAsset(run,kind) + '" alt="' + esc(label) + '"></section>').join('') +
        '<div class="flexwrap"><button type="button" class="btn" onclick="showInspectionGalleryList()">Back to Gallery</button></div></div>';
    inspectionGalleryDialog('Inspection Result', body);
}
