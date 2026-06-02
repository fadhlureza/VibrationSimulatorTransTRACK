const MAX_POINTS = 100;

function setupCanvas(canvasId) {
    const canvas = document.getElementById(canvasId);
    const ctx = canvas.getContext("2d");
    const data = Array(MAX_POINTS).fill(0);
    return { canvas, ctx, data };
}

const graphs = {
    pureG: setupCanvas("pureGCanvas"),
    calG: setupCanvas("calibratedGCanvas"),
    calMs2: setupCanvas("calibratedMs2Canvas"),
    // Buat combined, kita set manual 3 array terpisah
    combined: {
        canvas: document.getElementById("combinedCanvas"),
        ctx: document.getElementById("combinedCanvas").getContext("2d"),
        dataPot: Array(MAX_POINTS).fill(0),
        dataPwm: Array(MAX_POINTS).fill(0),
        dataMs2: Array(MAX_POINTS).fill(0)
    }
};

function drawGraph(graph, label, color, maxValue) {
    const { canvas, ctx, data } = graph;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // Draw grid
    ctx.strokeStyle = "#eee";
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    // Draw max value text
    ctx.fillStyle = "#888";
    ctx.font = "12px Arial";
    ctx.fillText(maxValue.toFixed(2) + " " + label, 5, 15);
    ctx.fillText("0 " + label, 5, h - 5);

    // Draw data line
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
    const { canvas, ctx, dataPot, dataPwm, dataMs2 } = graphs.combined;
    const w = canvas.width;
    const h = canvas.height;

    ctx.clearRect(0, 0, w, h);

    // Draw grid
    ctx.strokeStyle = "#eee";
    ctx.beginPath();
    for (let i = 0; i < 5; i++) {
        const y = i * (h / 4);
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
    }
    ctx.stroke();

    ctx.font = "12px Arial";
    ctx.fillStyle = "#FF9800"; ctx.fillText("Pot Raw", 10, 15);
    ctx.fillStyle = "#9C27B0"; ctx.fillText("PWM", 70, 15);
    ctx.fillStyle = "red";     ctx.fillText("m/s2", 110, 15);

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

    drawLine(dataPot, "#FF9800", 4095); // Potensio (Max 4095)
    drawLine(dataPwm, "#9C27B0", 255);  // PWM (Max 255)
    drawLine(dataMs2, "red", 40);       // m/s2 (Max 40)
}

function updateData(graph, newValue) {
    graph.data.push(newValue);
    graph.data.shift();
}

async function fetchData() {
    try {
        const response = await fetch('/api/data');
        const json = await response.json();

        updateData(graphs.pureG, json.vibration_g);
        updateData(graphs.calG, json.calibrated_g);
        updateData(graphs.calMs2, json.calibrated_ms2);

        graphs.combined.dataPot.push(json.pot_raw || 0);
        graphs.combined.dataPot.shift();
        
        graphs.combined.dataPwm.push(json.pwm_value || 0);
        graphs.combined.dataPwm.shift();
        
        graphs.combined.dataMs2.push(json.calibrated_ms2 || 0);
        graphs.combined.dataMs2.shift();

        drawGraph(graphs.pureG, "g", "blue", 4);
        drawGraph(graphs.calG, "g", "green", 4);
        drawGraph(graphs.calMs2, "m/s2", "red", 40);
        drawCombinedGraph();

    } catch (err) {
        console.error("Error fetching data:", err);
    }
    
    setTimeout(fetchData, 100);
}

drawGraph(graphs.pureG, "g", "blue", 4);
drawGraph(graphs.calG, "g", "green", 4);
drawGraph(graphs.calMs2, "m/s2", "red", 40);
drawCombinedGraph();

fetchData();