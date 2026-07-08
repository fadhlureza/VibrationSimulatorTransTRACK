const MAX_POINTS = 100;
const MAX_HISTORY_POINTS = 3600;

function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvas.getContext("2d");
    const data = Array(MAX_POINTS).fill(0);
    return { canvas, ctx, data };
}

const graphs = {
    targetG: setupCanvas("targetGCanvas"),
    pureG: setupCanvas("pureGCanvas"),
    calG: setupCanvas("calibratedGCanvas"),
    calMs2: setupCanvas("calibratedMs2Canvas"),
    freq: setupCanvas("freqCanvas"),
    combined: {
        canvas: document.getElementById("combinedCanvas"),
        ctx: document.getElementById("combinedCanvas").getContext("2d"),
        dataTarget: Array(MAX_POINTS).fill(0),
        dataPwm: Array(MAX_POINTS).fill(0),
        dataG: Array(MAX_POINTS).fill(0)
    },
    history: {
        canvas: document.getElementById("historyCanvas"),
        ctx: document.getElementById("historyCanvas").getContext("2d"),
        dataTarget: Array(MAX_HISTORY_POINTS).fill(0),
        dataPwm: Array(MAX_HISTORY_POINTS).fill(0),
        dataG: Array(MAX_HISTORY_POINTS).fill(0)
    }
};

let fetchCounter = 0;

function drawGraph(graph, label, color, maxValue) {
    const { canvas, ctx, data } = graph;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = "#eee";
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    ctx.fillStyle = "#888";
    ctx.font = "12px Arial";
    ctx.fillText(maxValue.toFixed(2) + " " + label, 5, 15);
    ctx.fillText("0 " + label, 5, h - 5);

    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    const dx = w / (MAX_POINTS - 1);
    const scale = (value) => h - (Math.min(value, maxValue) / maxValue) * h;

    for (let i = 0; i < MAX_POINTS; i++) {
        const x = i * dx;
        const y = scale(data[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

function drawCombinedGraph() {
    const { canvas, ctx, dataTarget, dataPwm, dataG } = graphs.combined;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);
    
    ctx.strokeStyle = "#eee";
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    ctx.font = "12px Arial";
    ctx.fillStyle = "#FF9800"; ctx.fillText("Target G", 10, 15);
    ctx.fillStyle = "#9C27B0"; ctx.fillText("PWM", 70, 15);
    ctx.fillStyle = "red";     ctx.fillText("G", 110, 15);

    const dx = w / (MAX_POINTS - 1);

    function drawLine(dataset, color, maxVal) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        const scale = (val) => h - (Math.min(val, maxVal) / maxVal) * h;
        
        for (let i = 0; i < MAX_POINTS; i++) {
            const x = i * dx;
            const y = scale(dataset[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    drawLine(dataTarget, "#FF9800", 16); // Target G (Max 16)
    drawLine(dataPwm, "#9C27B0", 255);  // PWM (Max 255)
    drawLine(dataG, "red", 16);        // G (Max 16)
}

function drawHistoryGraph() {
    const { canvas, ctx, dataTarget, dataPwm, dataG } = graphs.history;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    ctx.strokeStyle = "#eee";
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    const dx = w / (MAX_HISTORY_POINTS - 1);

    function drawLine(dataset, color, maxVal) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        const scale = (val) => h - (Math.min(val, maxVal) / maxVal) * h;
        
        for (let i = 0; i < MAX_HISTORY_POINTS; i++) {
            const x = i * dx;
            const y = scale(dataset[i]);
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }

    drawLine(dataTarget, "#FF9800", 16); // Target G
    drawLine(dataPwm, "#9C27B0", 255);  // PWM
    drawLine(dataG, "red", 16);        // G
}

function updateData(graph, newValue) {
    graph.data.push(newValue);
    graph.data.shift();
}

async function fetchData() {
    try {
        const response = await fetch('/api/data');
        const json = await response.json();
        
        document.getElementById('val-target').innerText = "[ " + json.target_g.toFixed(3) + " ]";
        document.getElementById('val-pure').innerText = "[ " + json.vibration_g.toFixed(3) + " ]";
        document.getElementById('val-calG').innerText = "[ " + json.calibrated_g.toFixed(3) + " ]";
        document.getElementById('val-calMs2').innerText = "[ " + json.calibrated_ms2.toFixed(3) + " ]";
        
        document.getElementById('val-targetG').innerText = json.target_g.toFixed(3);
        document.getElementById('val-pwm').innerText = json.pwm_value;
        document.getElementById('val-combG').innerText = json.calibrated_g.toFixed(3);

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

        drawGraph(graphs.targetG, "g", "orange", 16);
        drawGraph(graphs.pureG, "g", "blue", 16);
        drawGraph(graphs.calG, "g", "green", 16);
        drawGraph(graphs.calMs2, "m/s2", "red", 160);
        drawGraph(graphs.freq, "Hz", "purple", 100);
        drawCombinedGraph();

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
            document.getElementById('val-freq').innerText = json.dominant_freq_hz.toFixed(2);

            drawHistoryGraph();
            
            fetchCounter = 0;
        }

    } catch (err) {
        console.error("Error fetching data:", err);
    }
    
    setTimeout(fetchData, 100);
}

async function updatePID(){
    const kp = parseFloat(document.getElementById('input-kp').value) || 0;
    const ki = parseFloat(document.getElementById('input-ki').value) || 0;
    const kd = parseFloat(document.getElementById('input-kd').value) || 0;

    const statusText = document.getElementById('update-status');
    statusText.innerText = "Updating PID coefficients...";
    statusText.style.color = "orange";

    try {
        const response = await fetch('/api/update_pid', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ kp : kp, ki : ki, kd : kd })
        });
    
        if (response.ok) {
            statusText.innerText = "PID coefficients updated successfully!";
            statusText.style.color = "green";
            setTimeout(() => { statusText.innerText = ""; }, 2000);
        } else {
            statusText.innerText = "Failed to update PID coefficients.";
            statusText.style.color = "red";
        } 
    }
    catch (err) {
        console.error("Error updating PID coefficients:", err);
        statusText.innerText = "Error updating PID coefficients.";
        statusText.style.color = "red";
    }
}

drawGraph(graphs.pureG, "g", "blue", 16);
drawGraph(graphs.calG, "g", "green", 16);
drawGraph(graphs.targetG, "g", "orange", 16);
drawGraph(graphs.calMs2, "m/s2", "red", 160);
drawGraph(graphs.freq, "Hz", "purple", 100);

drawCombinedGraph();

fetchData();