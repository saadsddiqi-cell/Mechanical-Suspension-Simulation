/**
 * DYNA-RIDE Co-Simulator - Core Logic & Physics Engine
 * Author: Antigravity
 * Technology: Web Serial API, RK4 ODE Integration, Canvas Rendering
 */

// --- STATE VARIABLES ---
let simMode = 'quarter'; // 'quarter' or 'half'
let isConnected = false;
let serialPort = null;
let serialWriter = null;
let serialReader = null;
let keepReading = false;

// Physics Parameters (Adjustable via Sliders)
let mass = 5.0;       // kg (M)
let stiffness = 150.0; // N/m (K)
let damping = 25.0;    // N*s/m (B)
let velocity = 40.0;   // km/h (V) - for half-car wheelbase delay
let wheelbase = 1.8;   // meters (L)

// Wave Parameters (For Sine excitation)
let waveFreq = 1.5;   // Hz (f)
let waveAmp = 0.04;    // meters (A)

// Excitation state
let signalType = 'step'; // 'step', 'impulse', 'sine'
let time = 0.0;          // Simulation time in seconds
let dt = 0.005;         // Fixed time-step (200Hz)
let lastSerialSend = 0;  // Timestamp of last serial write (throttled to 50Hz)

// Quarter Car Simulation State Vector [x, v]
// x: chassis displacement (m), v: chassis velocity (m/s)
let qState = { x: 0.0, v: 0.0 };

// Half Car Simulation State Vector [z, z_dot, theta, theta_dot]
// z: heave displacement (m), z_dot: heave velocity (m/s)
// theta: pitch angle (rad), theta_dot: pitch velocity (rad/s)
let hState = {
    z: 0.0,
    z_dot: 0.0,
    theta: 0.0,
    theta_dot: 0.0
};

// Road profiles (height and velocity)
let roadState = {
    y: 0.0,      // Front (or single) road height (m)
    y_dot: 0.0,  // Front road velocity (m/s)
    y_r: 0.0,    // Rear road height (m)
    y_r_dot: 0.0 // Rear road velocity (m/s)
};

// Excitation triggers
let bumpTriggerTime = -999.0;

// Data logging for charts (sliding window)
const maxPoints = 500;
const roadDataHistory = [];
const carDataHistory = [];
const rearDataHistory = []; // Only for half-car mode

// --- UI ELEMENTS ---
const elements = {
    sliderMass: document.getElementById('sliderMass'),
    sliderStiffness: document.getElementById('sliderStiffness'),
    sliderDamping: document.getElementById('sliderDamping'),
    sliderSpeed: document.getElementById('sliderSpeed'),
    sliderFreq: document.getElementById('sliderFreq'),
    sliderAmp: document.getElementById('sliderAmp'),
    
    valMass: document.getElementById('valMass'),
    valStiffness: document.getElementById('valStiffness'),
    valDamping: document.getElementById('valDamping'),
    valSpeed: document.getElementById('valSpeed'),
    valFreq: document.getElementById('valFreq'),
    valAmp: document.getElementById('valAmp'),
    valZeta: document.getElementById('valZeta'),
    
    dampingRatioBox: document.getElementById('dampingRatioBox'),
    zetaBarFill: document.getElementById('zetaBarFill'),
    dampingStateText: document.getElementById('dampingStateText'),
    
    btnModeQuarter: document.getElementById('btnModeQuarter'),
    btnModeHalf: document.getElementById('btnModeHalf'),
    speedGroup: document.getElementById('speedGroup'),
    
    sineSettings: document.getElementById('sineSettings'),
    btnTrigger: document.getElementById('btnTrigger'),
    btnReset: document.getElementById('btnReset'),
    
    chartCanvas: document.getElementById('chartCanvas'),
    animCanvas: document.getElementById('animCanvas'),
    legendRear: document.getElementById('legendRear'),
    
    btnConnect: document.getElementById('btnConnect'),
    statusDot: document.getElementById('statusDot'),
    statusText: document.getElementById('statusText'),
    serialStatusBar: document.getElementById('serialStatusBar'),
    consoleLog: document.getElementById('consoleLog'),
    lblServoLeft: document.getElementById('lblServoLeft'),
    lblServoRight: document.getElementById('lblServoRight')
};

// --- INITIALIZATION ---
window.addEventListener('DOMContentLoaded', () => {
    setupSliders();
    setupButtons();
    resizeCanvases();
    window.addEventListener('resize', resizeCanvases);
    
    // Start the real-time simulation loop
    requestAnimationFrame(simulationLoop);
    
    logToConsole("[SYSTEM] Interface initialized. Simulation running at 200Hz.");
});

function resizeCanvases() {
    // Canvas sizing based on containers
    const chartContainer = elements.chartCanvas.parentElement;
    elements.chartCanvas.width = chartContainer.clientWidth;
    elements.chartCanvas.height = chartContainer.clientHeight - 20;

    const animContainer = elements.animCanvas.parentElement;
    elements.animCanvas.width = animContainer.clientWidth;
    elements.animCanvas.height = animContainer.clientHeight;
}

// --- CONSOLE LOGGER ---
function logToConsole(message) {
    const timestamp = new Date().toLocaleTimeString();
    elements.consoleLog.innerText += `\n[${timestamp}] ${message}`;
    elements.consoleLog.scrollTop = elements.consoleLog.scrollHeight;
}

// --- SLIDER CONFIGURATION ---
function setupSliders() {
    // Mass slider
    elements.sliderMass.addEventListener('input', (e) => {
        mass = parseFloat(e.target.value);
        elements.valMass.innerText = `${mass.toFixed(1)} kg`;
        updateDampingRatio();
    });

    // Stiffness slider
    elements.sliderStiffness.addEventListener('input', (e) => {
        stiffness = parseInt(e.target.value);
        elements.valStiffness.innerText = `${stiffness} N/m`;
        updateDampingRatio();
    });

    // Damping slider
    elements.sliderDamping.addEventListener('input', (e) => {
        damping = parseFloat(e.target.value);
        elements.valDamping.innerText = `${damping.toFixed(1)} N·s/m`;
        updateDampingRatio();
    });

    // Speed slider
    elements.sliderSpeed.addEventListener('input', (e) => {
        velocity = parseInt(e.target.value);
        elements.valSpeed.innerText = `${velocity} km/h`;
    });

    // Sine Frequency slider
    elements.sliderFreq.addEventListener('input', (e) => {
        waveFreq = parseFloat(e.target.value);
        elements.valFreq.innerText = `${waveFreq.toFixed(1)} Hz`;
    });

    // Sine Amplitude slider
    elements.sliderAmp.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        waveAmp = val / 100.0; // cm to meters
        elements.valAmp.innerText = `${val.toFixed(1)} cm`;
    });

    updateDampingRatio();
}

// Calculate and render Damping Ratio Zeta
function updateDampingRatio() {
    // Formula: Zeta = B / (2 * sqrt(K * M))
    const zeta = damping / (2 * Math.sqrt(stiffness * mass));
    elements.valZeta.innerText = zeta.toFixed(2);
    
    // Fill slider bar (0 to 2.0 max visual scale)
    const percentage = Math.min((zeta / 2.0) * 100, 100);
    elements.zetaBarFill.style.width = `${percentage}%`;
    
    // Update labels and colors
    elements.dampingRatioBox.className = 'damping-ratio-box';
    elements.zetaBarFill.className = 'ratio-bar-fill';
    elements.dampingStateText.className = 'damping-state';
    
    if (zeta < 0.95) {
        elements.dampingStateText.innerText = 'UNDERDAMPED (Oscillatory)';
        elements.dampingStateText.classList.add('state-underdamped');
        elements.zetaBarFill.style.backgroundColor = 'var(--accent-amber)';
        elements.zetaBarFill.style.boxShadow = '0 0 8px var(--accent-amber)';
    } else if (zeta >= 0.95 && zeta <= 1.05) {
        elements.dampingStateText.innerText = 'CRITICALLY DAMPED (Optimal)';
        elements.dampingStateText.classList.add('state-critical');
        elements.zetaBarFill.style.backgroundColor = 'var(--accent-emerald)';
        elements.zetaBarFill.style.boxShadow = '0 0 12px var(--accent-emerald)';
    } else {
        elements.dampingStateText.innerText = 'OVERDAMPED (Sluggish)';
        elements.dampingStateText.classList.add('state-overdamped');
        elements.zetaBarFill.style.backgroundColor = 'var(--accent-purple)';
        elements.zetaBarFill.style.boxShadow = '0 0 8px var(--accent-purple)';
    }
}

// --- BUTTONS SETUP ---
function setupButtons() {
    // Mode toggles
    elements.btnModeQuarter.addEventListener('click', () => {
        simMode = 'quarter';
        elements.btnModeQuarter.classList.add('active');
        elements.btnModeHalf.classList.remove('active');
        elements.speedGroup.classList.add('hidden');
        elements.legendRear.classList.add('hidden');
        resetSimulation();
        logToConsole("[SYSTEM] Switched to 1-DOF Quarter-Car Model.");
    });

    elements.btnModeHalf.addEventListener('click', () => {
        simMode = 'half';
        elements.btnModeHalf.classList.add('active');
        elements.btnModeQuarter.classList.remove('active');
        elements.speedGroup.classList.remove('hidden');
        elements.legendRear.classList.remove('hidden');
        resetSimulation();
        logToConsole("[SYSTEM] Switched to 2-DOF Half-Car (Pitching) Model.");
    });

    // Excitation Selectors
    const signalButtons = document.querySelectorAll('.btn-signal');
    signalButtons.forEach(btn => {
        btn.addEventListener('click', (e) => {
            signalButtons.forEach(b => b.classList.remove('active'));
            const clickedBtn = e.currentTarget;
            clickedBtn.classList.add('active');
            signalType = clickedBtn.dataset.signal;
            
            if (signalType === 'sine') {
                elements.sineSettings.classList.remove('hidden');
            } else {
                elements.sineSettings.classList.add('hidden');
            }
            
            resetSimulation();
            logToConsole(`[SYSTEM] Excitation profile changed to: ${signalType.toUpperCase()}`);
        });
    });

    // Bump Trigger
    elements.btnTrigger.addEventListener('click', () => {
        bumpTriggerTime = time;
        logToConsole("[SYSTEM] Manual road excitation triggered.");
    });

    // Reset Simulation
    elements.btnReset.addEventListener('click', () => {
        resetSimulation();
        logToConsole("[SYSTEM] Simulation reset completed.");
    });

    // Web Serial Connect
    elements.btnConnect.addEventListener('click', toggleSerialConnection);
}

function resetSimulation() {
    time = 0.0;
    bumpTriggerTime = -999.0;
    
    // Reset Quarter states
    qState.x = 0.0;
    qState.v = 0.0;
    
    // Reset Half states
    hState.z = 0.0;
    hState.z_dot = 0.0;
    hState.theta = 0.0;
    hState.theta_dot = 0.0;
    
    // Reset history buffers
    roadDataHistory.length = 0;
    carDataHistory.length = 0;
    rearDataHistory.length = 0;
}

// --- ROAD EXCITATION PROFILE ENGINE ---
// Generates the road profile y(t) and y_dot(t)
function getRoadProfile(t, positionOffset = 'front') {
    let tEval = t;
    
    // If evaluating the rear wheel in Half-Car mode, apply the wheelbase travel delay
    // delay = wheelbase / velocity_in_m_s
    if (simMode === 'half' && positionOffset === 'rear') {
        const velMS = velocity / 3.6; // km/h to m/s
        const timeDelay = wheelbase / velMS;
        tEval = t - timeDelay;
    }
    
    let y = 0.0;
    let y_dot = 0.0;
    
    if (signalType === 'sine') {
        // Continuous Sine Wave: y = A * sin(w * t)
        const omega = 2 * Math.PI * waveFreq;
        y = waveAmp * Math.sin(omega * tEval);
        y_dot = waveAmp * omega * Math.cos(omega * tEval);
    } else if (signalType === 'step') {
        // Pothole (Step input): smooth step using steep sigmoid from t = bumpTriggerTime
        if (bumpTriggerTime > 0 && tEval >= bumpTriggerTime) {
            const tLocal = tEval - bumpTriggerTime;
            const targetHeight = -0.05; // -5 cm deep pothole
            const riseTime = 0.05; // 50 ms transition for stability
            
            if (tLocal < riseTime) {
                // Cubic smoothstep transition
                const u = tLocal / riseTime;
                const smoothVal = u * u * (3 - 2 * u);
                const smoothDeriv = 6 * u * (1 - u) / riseTime;
                y = targetHeight * smoothVal;
                y_dot = targetHeight * smoothDeriv;
            } else {
                y = targetHeight;
                y_dot = 0.0;
            }
        }
    } else if (signalType === 'impulse') {
        // Sudden earthquake bump (Impulse / short pulse): Sine-squared pulse of 100ms
        if (bumpTriggerTime > 0 && tEval >= bumpTriggerTime) {
            const tLocal = tEval - bumpTriggerTime;
            const pulseDuration = 0.12; // 120 ms wide bump
            const bumpHeight = 0.06; // 6 cm high bump
            
            if (tLocal < pulseDuration) {
                // Sine squared pulse: y = H * sin^2(pi * t / D)
                const phase = Math.PI * tLocal / pulseDuration;
                y = bumpHeight * Math.sin(phase) * Math.sin(phase);
                // Derivative: y_dot = H * (pi/D) * sin(2 * pi * t / D)
                y_dot = bumpHeight * (Math.PI / pulseDuration) * Math.sin(2 * phase);
            } else {
                y = 0.0;
                y_dot = 0.0;
            }
        }
    }
    
    return { y, y_dot };
}

// --- RUNGE-KUTTA 4TH ORDER SOLVER ---
function runPhysicsStep() {
    time += dt;
    
    if (simMode === 'quarter') {
        // --- 1-DOF Quarter Car Model Solver ---
        // Equations:
        // x_dot = v
        // v_dot = ( K*(y - x) + B*(y_dot - v) ) / M
        
        // Define physics derivative function
        const getDerivatives = (s, tVal) => {
            const road = getRoadProfile(tVal, 'front');
            const forceSpring = stiffness * (road.y - s.x);
            const forceDamping = damping * (road.y_dot - s.v);
            const accelChassis = (forceSpring + forceDamping) / mass;
            return { dx: s.v, dv: accelChassis };
        };
        
        // RK4 Integration steps
        const s = { x: qState.x, v: qState.v };
        
        const k1 = getDerivatives(s, time);
        
        const s2 = { x: s.x + 0.5 * dt * k1.dx, v: s.v + 0.5 * dt * k1.dv };
        const k2 = getDerivatives(s2, time + 0.5 * dt);
        
        const s3 = { x: s.x + 0.5 * dt * k2.dx, v: s.v + 0.5 * dt * k2.dv };
        const k3 = getDerivatives(s3, time + 0.5 * dt);
        
        const s4 = { x: s.x + dt * k3.dx, v: s.v + dt * k3.dv };
        const k4 = getDerivatives(s4, time + dt);
        
        // Update state
        qState.x += (dt / 6.0) * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);
        qState.v += (dt / 6.0) * (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv);
        
        // Save current road excitation for graphics
        const road = getRoadProfile(time, 'front');
        roadState.y = road.y;
        roadState.y_dot = road.y_dot;
        
    } else {
        // --- 2-DOF Half Car Pitch & Heave Solver ---
        // z: vertical displacement (m)
        // theta: pitch angle (rad)
        // L_f = L_r = L/2
        // K_f = K_r = K/2, B_f = B_r = B/2
        // J = 0.12 * M * L^2 (Inertia)
        
        const L_half = wheelbase / 2.0;
        const K_susp = stiffness / 2.0;
        const B_susp = damping / 2.0;
        const J = 0.12 * mass * (wheelbase * wheelbase);
        
        const getDerivatives = (s, tVal) => {
            // Evaluates road under front and rear tires
            const roadF = getRoadProfile(tVal, 'front');
            const roadR = getRoadProfile(tVal, 'rear');
            
            // Deflections of front and rear wheels
            const z_front = s.z + L_half * s.theta;
            const z_rear = s.z - L_half * s.theta;
            
            const z_front_dot = s.z_dot + L_half * s.theta_dot;
            const z_rear_dot = s.z_dot - L_half * s.theta_dot;
            
            // Suspension forces
            const forceF = K_susp * (roadF.y - z_front) + B_susp * (roadF.y_dot - z_front_dot);
            const forceR = K_susp * (roadR.y - z_rear) + B_susp * (roadR.y_dot - z_rear_dot);
            
            // Dynamics
            const accelZ = (forceF + forceR) / mass;
            const accelTheta = ((forceF * L_half) - (forceR * L_half)) / J;
            
            return {
                dz: s.z_dot,
                dz_dot: accelZ,
                dtheta: s.theta_dot,
                dtheta_dot: accelTheta
            };
        };
        
        // RK4 Integration steps
        const s = { z: hState.z, z_dot: hState.z_dot, theta: hState.theta, theta_dot: hState.theta_dot };
        
        const k1 = getDerivatives(s, time);
        
        const s2 = {
            z: s.z + 0.5 * dt * k1.dz, z_dot: s.z_dot + 0.5 * dt * k1.dz_dot,
            theta: s.theta + 0.5 * dt * k1.dtheta, theta_dot: s.theta_dot + 0.5 * dt * k1.dtheta_dot
        };
        const k2 = getDerivatives(s2, time + 0.5 * dt);
        
        const s3 = {
            z: s.z + 0.5 * dt * k2.dz, z_dot: s.z_dot + 0.5 * dt * k2.dz_dot,
            theta: s.theta + 0.5 * dt * k2.dtheta, theta_dot: s.theta_dot + 0.5 * dt * k2.dtheta_dot
        };
        const k3 = getDerivatives(s3, time + 0.5 * dt);
        
        const s4 = {
            z: s.z + dt * k3.dz, z_dot: s.z_dot + dt * k3.dz_dot,
            theta: s.theta + dt * k3.dtheta, theta_dot: s.theta_dot + dt * k3.dtheta_dot
        };
        const k4 = getDerivatives(s4, time + dt);
        
        // Update states
        hState.z += (dt / 6.0) * (k1.dz + 2.0 * k2.dz + 2.0 * k3.dz + k4.dz);
        hState.z_dot += (dt / 6.0) * (k1.dz_dot + 2.0 * k2.dz_dot + 2.0 * k3.dz_dot + k4.dz_dot);
        hState.theta += (dt / 6.0) * (k1.dtheta + 2.0 * k2.dtheta + 2.0 * k3.dtheta + k4.dtheta);
        hState.theta_dot += (dt / 6.0) * (k1.dtheta_dot + 2.0 * k2.dtheta_dot + 2.0 * k3.dtheta_dot + k4.dtheta_dot);
        
        // Record front and rear road heights
        const roadF = getRoadProfile(time, 'front');
        const roadR = getRoadProfile(time, 'rear');
        roadState.y = roadF.y;
        roadState.y_dot = roadF.y_dot;
        roadState.y_r = roadR.y;
        roadState.y_r_dot = roadR.y_dot;
    }
}

// --- MAIN GRAPHICS & STREAM LOOP ---
function simulationLoop(timestamp) {
    // We execute physics updates at a fixed rate of 200Hz.
    // To match real-time, we estimate how many integration steps are needed since the last frame
    // but a simpler web implementation is to just run a fixed number of integration steps (e.g. 2 steps per animation frame at 60fps ~ 120Hz,
    // or run a time accumulator to run the physics loop at exactly 200Hz relative to real time).
    // Let's use a accurate accumulator for real-time.
    
    if (!simulationLoop.lastTime) simulationLoop.lastTime = timestamp;
    let elapsed = (timestamp - simulationLoop.lastTime) / 1000.0;
    simulationLoop.lastTime = timestamp;
    
    // Guard against large frame lags (e.g. background tab)
    if (elapsed > 0.1) elapsed = 0.1;
    
    simulationLoop.accumulator = (simulationLoop.accumulator || 0.0) + elapsed;
    
    while (simulationLoop.accumulator >= dt) {
        runPhysicsStep();
        simulationLoop.accumulator -= dt;
        
        // Add physics data to chart history
        if (simMode === 'quarter') {
            roadDataHistory.push(roadState.y);
            carDataHistory.push(qState.x);
            // Keep buffers clamped
            if (roadDataHistory.length > maxPoints) roadDataHistory.shift();
            if (carDataHistory.length > maxPoints) carDataHistory.shift();
        } else {
            roadDataHistory.push(roadState.y); // Front road height
            carDataHistory.push(hState.z);     // Vehicle center Heave
            rearDataHistory.push(roadState.y_r); // Rear road height
            
            if (roadDataHistory.length > maxPoints) roadDataHistory.shift();
            if (carDataHistory.length > maxPoints) carDataHistory.shift();
            if (rearDataHistory.length > maxPoints) rearDataHistory.shift();
        }
    }
    
    // Render canvases
    drawChart();
    drawAnimation();
    
    // Throttled USB Serial communication (run at 50Hz, i.e., every 20ms)
    const now = Date.now();
    if (now - lastSerialSend >= 20) {
        lastSerialSend = now;
        streamDataToHardware();
    }
    
    requestAnimationFrame(simulationLoop);
}

// --- RENDER REALTIME PLOT CANVAS ---
function drawChart() {
    const canvas = elements.chartCanvas;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    
    ctx.clearRect(0, 0, w, h);
    
    // Draw Grid
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 40) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
    }
    for (let y = 0; y < h; y += 30) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
    }
    
    // Baseline (0 position)
    const midY = h / 2;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, midY);
    ctx.lineTo(w, midY);
    ctx.stroke();
    
    // Scaling factors (Scale meters to canvas pixels)
    // Max displacement is approx +-12cm (+-0.12m)
    const scale = (h / 2) / 0.12; 
    
    // Plot Line Helper
    const plotLine = (data, color, shadowColor) => {
        if (data.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        
        if (shadowColor) {
            ctx.shadowColor = shadowColor;
            ctx.shadowBlur = 6;
        } else {
            ctx.shadowBlur = 0;
        }
        
        ctx.beginPath();
        const startX = w - (data.length * (w / maxPoints));
        ctx.moveTo(startX, midY - data[0] * scale);
        
        for (let i = 1; i < data.length; i++) {
            const currentX = w - ((data.length - i) * (w / maxPoints));
            ctx.lineTo(currentX, midY - data[i] * scale);
        }
        ctx.stroke();
        ctx.shadowBlur = 0; // reset
    };
    
    // Plot road, rear wheel, and car positions
    if (simMode === 'quarter') {
        plotLine(roadDataHistory, '#3b82f6', 'rgba(59, 130, 246, 0.5)'); // Neon blue road
        plotLine(carDataHistory, '#10b981', 'rgba(16, 185, 129, 0.5)');  // Neon green chassis
    } else {
        // Half car plots
        plotLine(roadDataHistory, '#3b82f6', 'rgba(59, 130, 246, 0.4)'); // Front road (blue)
        plotLine(rearDataHistory, '#f59e0b', 'rgba(245, 158, 11, 0.3)'); // Rear road (amber)
        plotLine(carDataHistory, '#10b981', 'rgba(16, 185, 129, 0.5)');  // Center heave (green)
    }
}

// --- RENDER 2D CAR DYNAMICS ANIMATION ---
function drawAnimation() {
    const canvas = elements.animCanvas;
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    
    ctx.clearRect(0, 0, w, h);
    
    // Draw grid background
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
    ctx.lineWidth = 1;
    for (let x = 0; x < w; x += 30) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
    }
    for (let y = 0; y < h; y += 30) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
    }

    const groundY = h * 0.75;
    
    if (simMode === 'quarter') {
        // --- Render Quarter Car View (Front profile) ---
        // Render road height
        const dispRoad = roadState.y; // m
        const dispChassis = qState.x; // m
        
        // Scale factor: 1 meter = 600 pixels
        const scale = 500;
        
        const wheelY = groundY - (dispRoad * scale) - 30; // Wheel center
        const bodyY = groundY - 120 - (dispChassis * scale); // Chassis bottom
        
        const centerX = w / 2;
        
        // 1. Draw Road surface
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(centerX - 120, groundY - (dispRoad * scale));
        ctx.lineTo(centerX + 120, groundY - (dispRoad * scale));
        ctx.stroke();
        
        // 2. Draw Wheel (Tire)
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.fillStyle = '#1e293b';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(centerX, wheelY, 30, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        
        // Wheel hub
        ctx.fillStyle = '#64748b';
        ctx.beginPath();
        ctx.arc(centerX, wheelY, 8, 0, 2 * Math.PI);
        ctx.fill();
        
        // 3. Draw Chassis body (Quarter car mass)
        ctx.fillStyle = '#312e81'; // dark indigo
        ctx.strokeStyle = '#6366f1'; // glowing violet border
        ctx.lineWidth = 3;
        ctx.shadowColor = 'rgba(99, 102, 241, 0.3)';
        ctx.shadowBlur = 10;
        
        // Draw main block
        ctx.beginPath();
        ctx.roundRect(centerX - 80, bodyY - 60, 160, 60, 8);
        ctx.fill();
        ctx.stroke();
        ctx.shadowBlur = 0; // reset
        
        // Draw mass text
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 12px Space Mono';
        ctx.textAlign = 'center';
        ctx.fillText(`M = ${mass.toFixed(1)}kg`, centerX, bodyY - 30);
        ctx.font = '10px Outfit';
        ctx.fillStyle = 'var(--text-secondary)';
        ctx.fillText("Quarter-Car Chassis", centerX, bodyY - 12);
        
        // 4. Draw Suspension Spring and Damper (Connecting wheel hub and chassis body)
        // Spring (left side of hub)
        drawSpring(ctx, centerX - 25, wheelY, centerX - 25, bodyY, 6, 25, '#f59e0b');
        
        // Damper (right side of hub)
        drawDamper(ctx, centerX + 25, wheelY, centerX + 25, bodyY, '#a855f7');
        
    } else {
        // --- Render Half Car View (Side profile) ---
        // L_half represents distance from center to wheels. Let's space wheels apart in the drawing.
        // Wheelbase on screen is 240px. Front wheel is at center + 120px, rear wheel is at center - 120px.
        const centerX = w / 2;
        const wBaseHalfPx = 120;
        
        const wheelFX = centerX + wBaseHalfPx;
        const wheelRX = centerX - wBaseHalfPx;
        
        // Get current dynamics displacements
        const dispZ = hState.z;     // Center of mass heave (m)
        const dispTheta = hState.theta; // Pitch angle (rad)
        
        // Scale meters to pixels
        const scale = 500;
        
        // Compute wheel heights from road profiles
        const roadF = roadState.y;   // Front road height
        const roadR = roadState.y_r; // Rear road height
        
        const wheelFY = groundY - (roadF * scale) - 25;
        const wheelRY = groundY - (roadR * scale) - 25;
        
        // Compute attachment points of suspension on chassis body
        // Body y position at front and rear
        const L_half_meters = wheelbase / 2.0;
        const dispChassisF = dispZ + L_half_meters * dispTheta;
        const dispChassisR = dispZ - L_half_meters * dispTheta;
        
        const bodyAttachFY = groundY - 110 - (dispChassisF * scale);
        const bodyAttachRY = groundY - 110 - (dispChassisR * scale);
        
        const bodyCenterY = groundY - 115 - (dispZ * scale);
        
        // 1. Draw Road profile
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 3;
        ctx.beginPath();
        // Left road section
        ctx.moveTo(0, groundY - (roadR * scale));
        ctx.lineTo(wheelRX + 50, groundY - (roadR * scale));
        ctx.stroke();
        
        ctx.strokeStyle = '#f59e0b'; // rear delay road color highlights
        ctx.beginPath();
        ctx.moveTo(wheelRX - 50, groundY - (roadR * scale));
        ctx.lineTo(wheelRX + 50, groundY - (roadR * scale));
        ctx.stroke();
        
        ctx.strokeStyle = '#3b82f6';
        ctx.beginPath();
        ctx.moveTo(wheelRX + 50, groundY - (roadR * scale));
        ctx.lineTo(wheelFX - 50, groundY - (roadF * scale)); // link road
        ctx.lineTo(w, groundY - (roadF * scale));
        ctx.stroke();
        
        // 2. Draw Front and Rear Wheels
        // Front Wheel (Right)
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.arc(wheelFX, wheelFY, 25, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#64748b';
        ctx.beginPath();
        ctx.arc(wheelFX, wheelFY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // Rear Wheel (Left)
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.beginPath();
        ctx.arc(wheelRX, wheelRY, 25, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = '#64748b';
        ctx.beginPath();
        ctx.arc(wheelRX, wheelRY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // 3. Draw Springs & Dampers
        // Front suspension (Front spring/damper offset slightly left/right of wheel hub)
        drawSpring(ctx, wheelFX - 15, wheelFY, wheelFX - 15, bodyAttachFY, 5, 20, '#f59e0b');
        drawDamper(ctx, wheelFX + 15, wheelFY, wheelFX + 15, bodyAttachFY, '#a855f7');
        
        // Rear suspension
        drawSpring(ctx, wheelRX - 15, wheelRY, wheelRX - 15, bodyAttachRY, 5, 20, '#f59e0b');
        drawDamper(ctx, wheelRX + 15, wheelRY, wheelRX + 15, bodyAttachRY, '#a855f7');
        
        // 4. Draw Vehicle Chassis (Tilted based on theta)
        ctx.save();
        ctx.translate(centerX, bodyCenterY);
        ctx.rotate(-dispTheta); // Pitch rotation
        
        ctx.fillStyle = '#1e1b4b'; // deep navy
        ctx.strokeStyle = '#6366f1';
        ctx.lineWidth = 3;
        ctx.shadowColor = 'rgba(99, 102, 241, 0.4)';
        ctx.shadowBlur = 10;
        
        // Draw double cab pickup truck body profile
        ctx.beginPath();
        ctx.moveTo(-150, 0);
        ctx.lineTo(-150, -35); // bed back
        ctx.lineTo(-60, -35);  // bed floor
        ctx.lineTo(-45, -65);  // rear window
        ctx.lineTo(40, -65);   // roof
        ctx.lineTo(65, -30);   // windshield
        ctx.lineTo(130, -30);  // hood
        ctx.lineTo(145, 0);    // front bumper
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.shadowBlur = 0;
        
        // Windows
        ctx.fillStyle = 'rgba(59, 130, 246, 0.35)';
        ctx.beginPath();
        ctx.moveTo(-35, -58);
        ctx.lineTo(35, -58);
        ctx.lineTo(55, -32);
        ctx.lineTo(-35, -32);
        ctx.closePath();
        ctx.fill();
        
        // Mass Center indicator
        ctx.fillStyle = '#10b981';
        ctx.beginPath();
        ctx.arc(0, -10, 5, 0, 2 * Math.PI);
        ctx.fill();
        
        // Overlay info text
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 11px Space Mono';
        ctx.textAlign = 'center';
        ctx.fillText(`M = ${mass.toFixed(1)}kg`, 0, -22);
        
        ctx.restore();
    }
}

// Draw a simple 2D zigzag spring
function drawSpring(ctx, x1, y1, x2, y2, coils, width, color) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.lineJoin = 'miter';
    
    // Draw spring assembly
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    
    // Top and bottom straight sections
    const l1 = 15;
    ctx.lineTo(x1, y1 - l1);
    
    const dy = (y2 + l1) - (y1 - l1);
    const step = dy / coils;
    
    let curX = x1;
    let curY = y1 - l1;
    
    for (let i = 0; i < coils; i++) {
        curY += step / 2;
        curX = x1 + (i % 2 === 0 ? width : -width);
        ctx.lineTo(curX, curY);
        
        curY += step / 2;
        curX = x1;
        ctx.lineTo(curX, curY);
    }
    
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.restore();
}

// Draw a piston damper (shock absorber)
function drawDamper(ctx, x1, y1, x2, y2, color) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 3;
    
    const dy = y2 - y1;
    const midY = y1 + dy * 0.45;
    
    // Bottom mounting and outer sleeve tube
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1, midY);
    ctx.stroke();
    
    // Outer tube cylinder sleeve
    ctx.lineWidth = 7;
    ctx.beginPath();
    ctx.moveTo(x1, midY);
    ctx.lineTo(x1, midY + dy * 0.2);
    ctx.stroke();
    
    // Top inner piston rod sliding into sleeve
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(x2, y2);
    ctx.lineTo(x2, midY + dy * 0.05);
    ctx.stroke();
    
    ctx.restore();
}

// --- USB SERIAL COMMUNICATIONS ---
async function toggleSerialConnection() {
    if (isConnected) {
        // Disconnect
        logToConsole("[SERIAL] Disconnecting hardware...");
        keepReading = false;
        
        if (serialReader) {
            try {
                await serialReader.cancel();
            } catch (err) {}
        }
        
        if (serialWriter) {
            try {
                serialWriter.releaseLock();
            } catch (err) {}
        }
        
        if (serialPort) {
            try {
                await serialPort.close();
            } catch (err) {}
        }
        
        setDisconnectedUI();
        logToConsole("[SERIAL] Hardware disconnected successfully.");
    } else {
        // Connect
        if (!("serial" in navigator)) {
            alert("Web Serial API is not supported in this browser. Please use Chrome, Edge, or Opera.");
            logToConsole("[ERROR] Web Serial API is unsupported on this browser.");
            return;
        }
        
        logToConsole("[SERIAL] Requesting COM port choice from user...");
        try {
            serialPort = await navigator.serial.requestPort();
            logToConsole("[SERIAL] Opening port at 115200 baud...");
            await serialPort.open({ baudRate: 115200 });
            
            serialWriter = serialPort.writable.getWriter();
            isConnected = true;
            setConnectedUI();
            logToConsole("[SERIAL] Connection established! Streaming coordinates.");
            
            // Start read loop in background for debug output from Arduino
            keepReading = true;
            readSerialData();
        } catch (err) {
            logToConsole(`[ERROR] Connection failed: ${err.message}`);
            setDisconnectedUI();
        }
    }
}

function setConnectedUI() {
    elements.statusDot.className = 'status-dot connected';
    elements.statusText.innerText = 'HARDWARE CONNECTED (115200)';
    elements.btnConnect.innerText = 'DISCONNECT';
    elements.btnConnect.classList.add('connected');
    elements.serialStatusBar.style.borderColor = 'var(--accent-emerald)';
}

function setDisconnectedUI() {
    elements.statusDot.className = 'status-dot disconnected';
    elements.statusText.innerText = 'HARDWARE DISCONNECTED';
    elements.btnConnect.innerText = 'CONNECT HARDWARE';
    elements.btnConnect.classList.remove('connected');
    elements.serialStatusBar.style.borderColor = 'rgba(255, 255, 255, 0.03)';
    isConnected = false;
    serialPort = null;
    serialWriter = null;
    serialReader = null;
    elements.lblServoLeft.innerText = "90°";
    elements.lblServoRight.innerText = "90°";
}

// Background loop to read serial prints from Arduino
async function readSerialData() {
    const textDecoder = new TextDecoderStream();
    const readableStreamClosed = serialPort.readable.pipeTo(textDecoder.writable);
    const reader = textDecoder.readable.getReader();
    serialReader = reader;
    
    try {
        let readBuffer = '';
        while (keepReading) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) {
                readBuffer += value;
                const lines = readBuffer.split('\n');
                readBuffer = lines.pop(); // keep last incomplete line
                for (const line of lines) {
                    const cleanLine = line.trim();
                    if (cleanLine.startsWith('[ARDUINO]')) {
                        logToConsole(cleanLine);
                    }
                }
            }
        }
    } catch (err) {
        logToConsole(`[SERIAL] Read error: ${err.message}`);
    } finally {
        reader.releaseLock();
    }
}

// Packs current displacement and sends values over USB serial
function streamDataToHardware() {
    if (!isConnected || !serialWriter) return;
    
    let leftTargetMeters = 0.0;
    let rightTargetMeters = 0.0;
    
    if (simMode === 'quarter') {
        // Quarter-car: both servos move in unison to display chassis bounce
        leftTargetMeters = qState.x;
        rightTargetMeters = qState.x;
    } else {
        // Half-car: front and rear wheel attachments drive respective servos
        const L_half_meters = wheelbase / 2.0;
        leftTargetMeters = hState.z + L_half_meters * hState.theta; // Front servo (Left)
        rightTargetMeters = hState.z - L_half_meters * hState.theta; // Rear servo (Right)
    }
    
    // --- Servo angle mapping ---
    // Physical system neutral displacement is 0.0m, which maps to 90 degrees.
    // Let's assume maximum physical range is +-10cm (+-0.10m).
    // Let's map +-10cm to +-45 degrees of servo rotation (i.e. from 45 to 135 degrees).
    // Mapping: angle = 90 + (displacement_meters / 0.10) * 45
    let leftAngle = Math.round(90 + (leftTargetMeters / 0.10) * 45);
    let rightAngle = Math.round(90 + (rightTargetMeters / 0.10) * 45);
    
    // Clamp to safe mechanical limits (40 to 140 degrees)
    leftAngle = Math.max(40, Math.min(140, leftAngle));
    rightAngle = Math.max(40, Math.min(140, rightAngle));
    
    // Display angles in the browser footer
    elements.lblServoLeft.innerHTML = `${leftAngle}&deg;`;
    elements.lblServoRight.innerHTML = `${rightAngle}&deg;`;
    
    // Send data packet over USB serial link
    // Protocol format: [leftAngle],[rightAngle]\n (e.g. "90,95\n")
    const encoder = new TextEncoder();
    const packet = `${leftAngle},${rightAngle}\n`;
    
    serialWriter.write(encoder.encode(packet)).catch(err => {
        logToConsole(`[ERROR] Write error: ${err.message}`);
        setDisconnectedUI();
    });
}
