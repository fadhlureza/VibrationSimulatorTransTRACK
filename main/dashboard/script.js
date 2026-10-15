const MAX_POINTS = 100;
const MAX_HISTORY_POINTS = 3600;

function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return null;
    const ctx = canvas.getContext("2d");
    const data = Array(MAX_POINTS).fill(0);
    return { canvas, ctx, data };
}

const graphs = {
    // Pitch Axis Graphs
    pitchTrack: {
        canvas: document.getElementById("pitchAngleCanvas"),
        ctx: document.getElementById("pitchAngleCanvas")?.getContext("2d"),
        dataTarget: Array(MAX_POINTS).fill(0),
        dataSlew: Array(MAX_POINTS).fill(0),
        dataLive: Array(MAX_POINTS).fill(0)
    },
    pitchPwm: {
        canvas: document.getElementById("pitchPwmCanvas"),
        ctx: document.getElementById("pitchPwmCanvas")?.getContext("2d"),
        dataPwm: Array(MAX_POINTS).fill(0)
    },

    // Vibration Axis Graphs (Original preserved)
    targetG: setupCanvas("targetGCanvas"),
    pureG: setupCanvas("pureGCanvas"),
    calG: setupCanvas("calibratedGCanvas"),
    calMs2: setupCanvas("calibratedMs2Canvas"),
    freq: setupCanvas("freqCanvas"),

    // Multi-Channel Combined Graphs (Original preserved)
    combined: {
        canvas: document.getElementById("combinedCanvas"),
        ctx: document.getElementById("combinedCanvas")?.getContext("2d"),
        dataTarget: Array(MAX_POINTS).fill(0),
        dataPwm: Array(MAX_POINTS).fill(0),
        dataG: Array(MAX_POINTS).fill(0)
    },
    history: {
        canvas: document.getElementById("historyCanvas"),
        ctx: document.getElementById("historyCanvas")?.getContext("2d"),
        dataTarget: Array(MAX_HISTORY_POINTS).fill(0),
        dataPwm: Array(MAX_HISTORY_POINTS).fill(0),
        dataG: Array(MAX_HISTORY_POINTS).fill(0)
    }
};

let fetchCounter = 0;
let isConnected = false;
let currentTab = "monitor";

// Tab Switching
function switchTab(tabId) {
    currentTab = tabId;
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.nav-link').forEach(el => el.classList.remove('active'));

    const activeTab = document.getElementById('tab-' + tabId);
    const activeNav = document.getElementById('btn-nav-' + tabId);
    const pageHeading = document.getElementById('page-heading');

    if (activeTab) activeTab.classList.add('active');
    if (activeNav) activeNav.classList.add('active');

    if (tabId === 'monitor') {
        pageHeading.innerText = "Data Monitor";
    } else if (tabId === 'config') {
        pageHeading.innerText = "Configuration";
    }
}

// Dark/Light Theme Toggle (matches moon icon in Image 3)
function toggleTheme() {
    const body = document.body;
    const btn = document.getElementById('theme-btn');
    if (body.classList.contains('theme-dark')) {
        body.classList.remove('theme-dark');
        body.classList.add('theme-light');
        btn.innerText = "🌙";
    } else {
        body.classList.remove('theme-light');
        body.classList.add('theme-dark');
        btn.innerText = "☀️";
    }
    redrawAllGraphs();
}

function toggleSidebar() {
    const sidebar = document.getElementById('app-sidebar');
    if (!sidebar) return;

    const isCollapsed = sidebar.classList.toggle('collapsed');

    try {
        localStorage.setItem('vst_sidebar_collapsed', isCollapsed ? "1" : "0");
    } catch (e) {}

    // Allow CSS transition to finish before recalculating canvas widths
    setTimeout(() => {
        redrawAllGraphs();
    }, 320);
}

// Helpers for theme-aware grid line colors
function isDarkTheme() {
    return document.body.classList.contains('theme-dark');
}

function getGridColor() {
    return isDarkTheme() ? "rgba(255, 255, 255, 0.07)" : "rgba(0, 0, 0, 0.06)";
}

function getZeroLineColor() {
    return isDarkTheme() ? "rgba(255, 255, 255, 0.3)" : "rgba(0, 0, 0, 0.25)";
}

function getTextColor() {
    return isDarkTheme() ? "#94A3B8" : "#64748B";
}

function ensureCanvasDimensions(canvas) {
    if (!canvas) return;
    const parentWidth = canvas.parentElement ? canvas.parentElement.clientWidth : 0;
    const targetW = parentWidth > 50 ? parentWidth : 800;
    if (Math.abs(canvas.width - targetW) > 2) {
        canvas.width = targetW;
    }
}

// Draw 0-to-Max Graph (Unsigned)
function drawGraph(graph, label, color, maxValue) {
    if (!graph || !graph.canvas || !graph.ctx) return;
    ensureCanvasDimensions(graph.canvas);
    const { canvas, ctx, data } = graph;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // Grid lines
    ctx.strokeStyle = getGridColor();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i <= 4; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    // Scale labels
    ctx.fillStyle = getTextColor();
    ctx.font = "11px -apple-system, sans-serif";
    ctx.fillText(maxValue.toFixed(1) + " " + label, 8, 14);
    ctx.fillText("0 " + label, 8, h - 6);

    // Graph line
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    const dx = w / (MAX_POINTS - 1);
    const scale = (value) => h - (Math.min(Math.max(value, 0), maxValue) / maxValue) * (h - 12) - 6;

    for (let i = 0; i < MAX_POINTS; i++) {
        const x = i * dx;
        const y = scale(data[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

// Draw Signed Center-Zero Graph (-Max to +Max)
function drawSignedGraph(canvas, ctx, datasets, maxRange, unitLabel) {
    if (!canvas || !ctx) return;
    ensureCanvasDimensions(canvas);
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // Horizontal grid lines
    ctx.strokeStyle = getGridColor();
    ctx.lineWidth = 1;
    ctx.beginPath();
    
    [0.05 * h, 0.275 * h, 0.725 * h, 0.95 * h].forEach(y => {
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    });
    ctx.stroke();

    // Center Zero Line (dashed)
    ctx.strokeStyle = getZeroLineColor();
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();
    ctx.setLineDash([]); // Reset dash

    // Labels
    ctx.fillStyle = getTextColor();
    ctx.font = "11px -apple-system, sans-serif";
    ctx.fillText("+" + maxRange.toFixed(0) + " " + unitLabel, 8, 14);
    ctx.fillText("0 " + unitLabel, 8, h / 2 - 4);
    ctx.fillText("-" + maxRange.toFixed(0) + " " + unitLabel, 8, h - 6);

    const dx = w / (MAX_POINTS - 1);
    const scaleY = (val) => {
        const clamped = Math.max(-maxRange, Math.min(maxRange, val));
        return (h / 2) - (clamped / maxRange) * (h * 0.44);
    };

    // Draw datasets
    datasets.forEach(ds => {
        ctx.strokeStyle = ds.color;
        ctx.lineWidth = ds.lineWidth || 2;
        if (ds.dashed) ctx.setLineDash([3, 3]);
        else ctx.setLineDash([]);

        ctx.beginPath();
        for (let i = 0; i < MAX_POINTS; i++) {
            const x = i * dx;
            const y = scaleY(ds.data[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    });
    ctx.setLineDash([]);
}

// Draw Multi-Channel Combined Graph
function drawCombinedGraph() {
    const { canvas, ctx, dataTarget, dataPwm, dataG } = graphs.combined;
    if (!canvas || !ctx) return;
    ensureCanvasDimensions(canvas);
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);
    
    // Grid
    ctx.strokeStyle = getGridColor();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i <= 4; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    const dx = w / (MAX_POINTS - 1);

    function drawLine(dataset, color, maxVal, width) {
        ctx.strokeStyle = color;
        ctx.lineWidth = width || 2;
        ctx.beginPath();
        const scale = (val) => h - (Math.min(Math.max(val, 0), maxVal) / maxVal) * (h - 14) - 7;
        
        for (let i = 0; i < MAX_POINTS; i++) {
            const x = i * dx;
            const y = scale(dataset[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    drawLine(dataTarget, "#D97706", 16, 2);   // Target G (Amber)
    drawLine(dataPwm, "#7C3AED", 255, 1.5);   // PWM (Purple)
    drawLine(dataG, "#E51E25", 16, 2);        // Actual G (TransTRACK Red)
}

// Draw 1-Hour Rolling History Graph
function drawHistoryGraph() {
    const { canvas, ctx, dataTarget, dataPwm, dataG } = graphs.history;
    if (!canvas || !ctx) return;
    ensureCanvasDimensions(canvas);
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = getGridColor();
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 0; i <= 4; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    const dx = w / (MAX_HISTORY_POINTS - 1);

    function drawLine(dataset, color, maxVal) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        const scale = (val) => h - (Math.min(Math.max(val, 0), maxVal) / maxVal) * (h - 14) - 7;
        
        for (let i = 0; i < MAX_HISTORY_POINTS; i++) {
            const x = i * dx;
            const y = scale(dataset[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    drawLine(dataTarget, "#D97706", 16);
    drawLine(dataPwm, "#7C3AED", 255);
    drawLine(dataG, "#E51E25", 16);
}

function updateData(graph, newValue) {
    if (!graph || !graph.data) return;
    graph.data.push(newValue);
    graph.data.shift();
}

function redrawAllGraphs() {
    drawSignedGraph(graphs.pitchTrack.canvas, graphs.pitchTrack.ctx, [
        { data: graphs.pitchTrack.dataTarget, color: "#D97706", lineWidth: 2, dashed: true },
        { data: graphs.pitchTrack.dataSlew,   color: "#E51E25", lineWidth: 1.5, dashed: false },
        { data: graphs.pitchTrack.dataLive,   color: "#0284C7", lineWidth: 2.2, dashed: false }
    ], 18, "°");

    drawSignedGraph(graphs.pitchPwm.canvas, graphs.pitchPwm.ctx, [
        { data: graphs.pitchPwm.dataPwm, color: "#E51E25", lineWidth: 2, dashed: false }
    ], 1023, "PWM");

    drawGraph(graphs.targetG, "g", "#D97706", 16);
    drawGraph(graphs.pureG, "g", "#0284C7", 16);
    drawGraph(graphs.calG, "g", "#10B981", 16);
    drawGraph(graphs.calMs2, "m/s²", "#E51E25", 160);
    drawGraph(graphs.freq, "Hz", "#7C3AED", 100);
    drawCombinedGraph();
    drawHistoryGraph();
}

// Fetch Initial PID values from ESP32 on page load
async function fetchInitialPIDs() {
    try {
        const res = await fetch('/api/pids');
        if (res.ok) {
            const data = await res.json();
            if (data.vib_kp !== undefined) {
                document.getElementById('input-kp').value = data.vib_kp.toFixed(3);
                document.getElementById('input-ki').value = data.vib_ki.toFixed(3);
                document.getElementById('input-kd').value = data.vib_kd.toFixed(3);
            }
            if (data.pitch_kp !== undefined) {
                document.getElementById('input-pitch-kp').value = data.pitch_kp.toFixed(2);
                document.getElementById('input-pitch-ki').value = data.pitch_ki.toFixed(2);
                document.getElementById('input-pitch-kd').value = data.pitch_kd.toFixed(2);
            }
        }
    } catch (e) {
        console.warn("Could not load initial PID coefficients:", e);
    }
}

// Polling telemetry loop (100 ms)
async function fetchData() {
    try {
        const response = await fetch('/api/data');
        if (!response.ok) throw new Error("HTTP error " + response.status);
        const json = await response.json();
        
        // Update connection status in sidebar
        if (!isConnected) {
            isConnected = true;
            const dot = document.getElementById('conn-dot');
            dot.className = "status-dot online";
            document.getElementById('conn-text').innerText = "ESP32 ONLINE";
        }

        // 1. Update Top Status Pills
        const stateStr = json.sys_state || "UNKNOWN";
        const pillState = document.getElementById('pill-state');
        if (pillState) {
            pillState.className = "status-pill state-" + stateStr.toLowerCase();
            const textEl = document.getElementById('val-pill-state');
            if (textEl) textEl.innerText = "State: " + stateStr;
            const stateIcons = {
                "CALIBRATING": "⏳",
                "HOMING": "📐",
                "RUNNING": "⚡",
                "STOPPED": "⏹",
                "FAULT": "🚨"
            };
            const iconEl = document.getElementById('icon-pill-state');
            if (iconEl) iconEl.innerText = stateIcons[stateStr.toUpperCase()] || "⚡";
        }

        document.getElementById('val-pill-mode').innerText = json.mode_str ? ("Mode: " + json.mode_str) : ("Mode " + json.mode);
        document.getElementById('val-pill-dir').innerText = "DIR: " + json.dir + (json.dir === "FWD" ? " (+)" : " (-)");

        const pitchSign = (json.pitch_angle >= 0 ? "+" : "");
        document.getElementById('val-pill-pitch').innerText = "Pitch: " + pitchSign + json.pitch_angle.toFixed(2) + "°";
        document.getElementById('val-pill-vib').innerText = "Vib: " + json.calibrated_g.toFixed(3) + " g";

        const pillDwellText = document.getElementById('val-pill-dwell');
        if (json.mode === 1) {
            if (json.dwell_rem > 0.05) {
                pillDwellText.innerText = "Dwell: " + json.dwell_rem.toFixed(1) + "s / " + json.dwell_total.toFixed(0) + "s";
            } else {
                pillDwellText.innerText = "Sweeping (" + json.pot1_deg.toFixed(1) + "°)";
            }
        } else {
            pillDwellText.innerText = "Dwell: OFF";
        }

        // 2. Update Card Header Real-Time Text
        document.getElementById('val-pitch-live').innerText = pitchSign + json.pitch_angle.toFixed(2) + "°";
        document.getElementById('val-pitch-target').innerText = (json.pitch_target >= 0 ? "+" : "") + json.pitch_target.toFixed(2) + "°";
        document.getElementById('val-pitch-slew').innerText = (json.pitch_slew >= 0 ? "+" : "") + json.pitch_slew.toFixed(2) + "°";

        document.getElementById('val-pitch-pwm').innerText = (json.pitch_pwm >= 0 ? "+" : "") + json.pitch_pwm;
        document.getElementById('val-pitch-pwm-dir').innerText = (json.pitch_pwm > 0 ? "FWD (Right Up)" : (json.pitch_pwm < 0 ? "REV (Left Up)" : "HOLD"));

        document.getElementById('val-target').innerText = "[ " + json.target_g.toFixed(3) + " g ]";
        const safetyBadge = document.getElementById('badge-safety-cap');
        const specVibSafety = document.getElementById('spec-vib-safety');
        const specVibSafetyVal = document.getElementById('spec-vib-safety-val');
        const isVibSafetyOn = !!json.vib_safety;

        if (isVibSafetyOn) {
            const maxGStr = ((typeof json.vib_safety_max_g === 'number') ? json.vib_safety_max_g : 1.0).toFixed(3);
            if (safetyBadge) {
                safetyBadge.style.display = "inline-block";
                safetyBadge.innerText = "🛡️ Cap: " + maxGStr + " g";
            }
            if (specVibSafety) {
                specVibSafety.style.display = "flex";
            }
            if (specVibSafetyVal) {
                specVibSafetyVal.innerText = maxGStr + " g (Max)";
            }
        } else {
            if (safetyBadge) {
                safetyBadge.style.display = "none";
            }
            if (specVibSafety) {
                specVibSafety.style.display = "none";
            }
        }
        document.getElementById('val-pure').innerText = "[ " + json.vibration_g.toFixed(3) + " g ]";
        document.getElementById('val-calG').innerText = "[ " + json.calibrated_g.toFixed(3) + " g ]";
        document.getElementById('val-calMs2').innerText = "[ " + json.calibrated_ms2.toFixed(3) + " m/s² ]";
        document.getElementById('val-freq').innerText = "[ " + json.dominant_freq_hz.toFixed(2) + " Hz ]";
        
        document.getElementById('val-targetG').innerText = json.target_g.toFixed(3);
        document.getElementById('val-pwm').innerText = json.pwm_value;
        document.getElementById('val-combG').innerText = json.calibrated_g.toFixed(3);

        // 3. Update Pitch Axis Graphs Data
        graphs.pitchTrack.dataTarget.push(json.pitch_target || 0);
        graphs.pitchTrack.dataTarget.shift();

        graphs.pitchTrack.dataSlew.push(json.pitch_slew || 0);
        graphs.pitchTrack.dataSlew.shift();

        graphs.pitchTrack.dataLive.push(json.pitch_angle || 0);
        graphs.pitchTrack.dataLive.shift();

        graphs.pitchPwm.dataPwm.push(json.pitch_pwm || 0);
        graphs.pitchPwm.dataPwm.shift();

        // Draw Pitch Graphs
        drawSignedGraph(graphs.pitchTrack.canvas, graphs.pitchTrack.ctx, [
            { data: graphs.pitchTrack.dataTarget, color: "#D97706", lineWidth: 2, dashed: true },
            { data: graphs.pitchTrack.dataSlew,   color: "#E51E25", lineWidth: 1.5, dashed: false },
            { data: graphs.pitchTrack.dataLive,   color: "#0284C7", lineWidth: 2.2, dashed: false }
        ], 18, "°");

        drawSignedGraph(graphs.pitchPwm.canvas, graphs.pitchPwm.ctx, [
            { data: graphs.pitchPwm.dataPwm, color: "#E51E25", lineWidth: 2, dashed: false }
        ], 1023, "PWM");

        // 4. Update Vibration Graphs Data
        updateData(graphs.targetG, json.target_g);
        updateData(graphs.pureG, json.vibration_g);
        updateData(graphs.calG, json.calibrated_g);
        updateData(graphs.calMs2, json.calibrated_ms2);
        updateData(graphs.freq, json.dominant_freq_hz);

        graphs.combined.dataTarget.push(json.target_g || 0);
        graphs.combined.dataTarget.shift();
        
        graphs.combined.dataPwm.push(json.pwm_value || 0);
        graphs.combined.dataPwm.shift();
        
        graphs.combined.dataG.push(json.calibrated_g || 0);
        graphs.combined.dataG.shift();

        const targetMaxG = (isVibSafetyOn && json.vib_safety_max_g > 0) ? Math.max(1.2, json.vib_safety_max_g * 1.25) : 16;
        drawGraph(graphs.targetG, "g", "#D97706", targetMaxG);
        drawGraph(graphs.pureG, "g", "#0284C7", 16);
        drawGraph(graphs.calG, "g", "#10B981", 16);
        drawGraph(graphs.calMs2, "m/s²", "#E51E25", 160);
        drawGraph(graphs.freq, "Hz", "#7C3AED", 100);
        drawCombinedGraph();

        // 5. Update 1-Hour Rolling History Graph (1 Hz)
        fetchCounter++;
        if (fetchCounter >= 10) {
            graphs.history.dataTarget.push(json.target_g || 0);
            graphs.history.dataTarget.shift();
            
            graphs.history.dataPwm.push(json.pwm_value || 0);
            graphs.history.dataPwm.shift();
            
            graphs.history.dataG.push(json.calibrated_g || 0);
            graphs.history.dataG.shift();

            document.getElementById('val-histTarget').innerText = json.target_g.toFixed(3);
            document.getElementById('val-histPwm').innerText = json.pwm_value;
            document.getElementById('val-histG').innerText = json.calibrated_g.toFixed(3);

            drawHistoryGraph();
            fetchCounter = 0;
        }

    } catch (err) {
        console.error("Telemetry fetch error:", err);
        if (isConnected) {
            isConnected = false;
            const dot = document.getElementById('conn-dot');
            dot.className = "status-dot offline";
            document.getElementById('conn-text').innerText = "DISCONNECTED";
        }
    }
    
    setTimeout(fetchData, 100);
}

// Update Vibration Motor PID
async function updatePID() {
    const kp = parseFloat(document.getElementById('input-kp').value) || 0;
    const ki = parseFloat(document.getElementById('input-ki').value) || 0;
    const kd = parseFloat(document.getElementById('input-kd').value) || 0;

    const statusText = document.getElementById('update-status');
    statusText.innerText = "Saving Vibration PID...";
    statusText.style.color = "#D97706";

    try {
        const response = await fetch('/api/update_pid', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ kp: kp, ki: ki, kd: kd })
        });
    
        if (response.ok) {
            statusText.innerText = "✓ Vibration PID updated successfully!";
            statusText.style.color = "#10B981";
            setTimeout(() => { statusText.innerText = ""; }, 3000);
        } else {
            statusText.innerText = "✗ Failed to update Vibration PID";
            statusText.style.color = "#E51E25";
        } 
    } catch (err) {
        console.error("Error updating vibration PID:", err);
        statusText.innerText = "✗ Network error!";
        statusText.style.color = "#E51E25";
    }
}

// Update Pitch Motor PID
async function updatePitchPID() {
    const kp = parseFloat(document.getElementById('input-pitch-kp').value) || 0;
    const ki = parseFloat(document.getElementById('input-pitch-ki').value) || 0;
    const kd = parseFloat(document.getElementById('input-pitch-kd').value) || 0;

    const statusText = document.getElementById('update-pitch-status');
    statusText.innerText = "Saving Pitch PID...";
    statusText.style.color = "#D97706";

    try {
        const response = await fetch('/api/update_pitch_pid', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ kp: kp, ki: ki, kd: kd })
        });
    
        if (response.ok) {
            statusText.innerText = "✓ Pitch PID updated successfully!";
            statusText.style.color = "#10B981";
            setTimeout(() => { statusText.innerText = ""; }, 3000);
        } else {
            statusText.innerText = "✗ Failed to update Pitch PID";
            statusText.style.color = "#E51E25";
        } 
    } catch (err) {
        console.error("Error updating pitch PID:", err);
        statusText.innerText = "✗ Network error!";
        statusText.style.color = "#E51E25";
    }
}

// Restore saved sidebar collapsed state on load
try {
    const savedCollapsed = localStorage.getItem('vst_sidebar_collapsed');
    if (savedCollapsed === "1") {
        const sidebar = document.getElementById('app-sidebar');
        if (sidebar) sidebar.classList.add('collapsed');
    }
} catch (e) {}

// Responsive resize listener for graphs
let resizeTimeout = null;
window.addEventListener('resize', () => {
    clearTimeout(resizeTimeout);
    resizeTimeout = setTimeout(() => {
        redrawAllGraphs();
    }, 150);
});

// Initial Canvas Pre-Draw
redrawAllGraphs();

// Load initial PID parameters and start telemetry polling
fetchInitialPIDs();
fetchData();