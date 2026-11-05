/**
 * FastDrink Telemetry CSV Replay & Robot Modelizer (app.js)
 * Modélisation cinématique 2D (odométrie Ackermann, roues directrices, capteurs ligne, sonar)
 * + Graphiques détaillés synchronisés (FSM, Moteurs, Direction, Capteurs, PID, Diagnostics)
 */

// ==============================================================================
// 1. DÉFINITION DES CONSTANTES & ÉTATS FASTDRINK
// ==============================================================================
const FSM_STATES = {
    0: { name: "Init", color: "#64748b" },
    1: { name: "Unarmed", color: "#eab308" },
    2: { name: "Armed", color: "#06b6d4" },
    3: { name: "LineFollowingForward", color: "#10b981" },
    4: { name: "CollisionRisk", color: "#f97316" },
    5: { name: "Stopped", color: "#ef4444" },
    6: { name: "CollisionOccured", color: "#991b1b" },
    7: { name: "RightDeviation", color: "#3b82f6" },
    8: { name: "LeftDeviation", color: "#8b5cf6" },
    9: { name: "Reverse", color: "#ec4899" },
    10: { name: "LineSearching", color: "#d97706" },
    11: { name: "LineDetected", color: "#14b8a6" },
    12: { name: "LineSearchingTimedOut", color: "#f43f5e" },
    13: { name: "Abort", color: "#7f1d1d" },
    14: { name: "Failure", color: "#450a0a" },
    255: { name: "Undefined", color: "#475569" }
};

const MOTOR_STATES = {
    0: "Init",
    1: "Idle",
    2: "Running",
    3: "Abort"
};

// Paramètres physiques du robot PiCar-S (en mètres)
const ROBOT_WHEELBASE = 0.14;   // Empattement L (14 cm)
const ROBOT_WIDTH = 0.13;       // Voie W (13 cm)
const ROBOT_LENGTH = 0.20;      // Longueur châssis (20 cm)
const SENSOR_BAR_OFFSET = 0.08; // Capteurs placés à +8 cm en avant de l'essieu avant
const MAX_VELOCITY_MPS = 1.0;   // Vitesse nominale maximale à throttle 1.0 (m/s)

// ==============================================================================
// 2. ÉTAT GLOBAL DE L'APPLICATION
// ==============================================================================
const state = {
    data: [],           // Lignes de données CSV brutes et typées
    poses: [],          // Trajectoire calculée [ {x, y, theta, v, dist, timeSec} ]
    totalFrames: 0,
    currentIndex: 0,
    isPlaying: false,
    playbackSpeed: 1.0,
    isLooping: false,
    lastFrameTime: null,
    
    // Mode d'affichage des capteurs de ligne
    lineSensorMode: "norm", // "norm", "raw", "corr"
    
    // Caméra Canvas
    camera: {
        x: 0,
        y: 0,
        zoom: 160,       // Pixels par mètre
        followRobot: true,
        isDragging: false,
        dragStartX: 0,
        dragStartY: 0
    },

    // Graphiques Chart.js
    charts: {},
    summary: {}
};

// ==============================================================================
// 3. INITIALISATION & ÉVÉNEMENTS
// ==============================================================================
window.addEventListener("DOMContentLoaded", () => {
    setupCanvas();
    setupEventListeners();
    initCharts();
    
    // Charger automatiquement l'échantillon au premier lancement
    loadSampleData();
});

function setupEventListeners() {
    // Fichier CSV
    const fileInput = document.getElementById("csv-file-input");
    fileInput.addEventListener("change", handleFileSelect);

    const btnSample = document.getElementById("btn-load-sample");
    btnSample.addEventListener("click", loadSampleData);

    const btnExportKpi = document.getElementById("btn-export-kpi");
    btnExportKpi.addEventListener("click", exportKpiReport);

    // Drag and Drop
    const dropZone = document.getElementById("drop-zone");
    dropZone.addEventListener("dragover", (e) => {
        e.preventDefault();
        dropZone.classList.add("dragover");
    });
    dropZone.addEventListener("dragleave", () => {
        dropZone.classList.remove("dragover");
    });
    dropZone.addEventListener("drop", (e) => {
        e.preventDefault();
        dropZone.classList.remove("dragover");
        if (e.dataTransfer.files && e.dataTransfer.files[0]) {
            parseCSVFile(e.dataTransfer.files[0]);
        }
    });

    // Contrôles Rejeu
    document.getElementById("btn-play-pause").addEventListener("click", togglePlayPause);
    document.getElementById("btn-step-prev").addEventListener("click", () => stepFrame(-1));
    document.getElementById("btn-step-next").addEventListener("click", () => stepFrame(1));
    document.getElementById("btn-reset").addEventListener("click", () => seekFrame(0));
    
    const btnLoop = document.getElementById("btn-loop");
    btnLoop.addEventListener("click", () => {
        state.isLooping = !state.isLooping;
        btnLoop.classList.toggle("active", state.isLooping);
    });

    // Boutons de vitesse
    document.querySelectorAll(".speed-btn").forEach(btn => {
        btn.addEventListener("click", (e) => {
            document.querySelectorAll(".speed-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            state.playbackSpeed = parseFloat(btn.dataset.speed);
        });
    });

    // Scrubber slider
    const scrubber = document.getElementById("scrubber");
    scrubber.addEventListener("input", (e) => {
        seekFrame(parseInt(e.target.value, 10));
    });

    // Raccourcis Clavier
    window.addEventListener("keydown", (e) => {
        if (e.target.tagName === "INPUT") return;
        if (e.code === "Space") {
            e.preventDefault();
            togglePlayPause();
        } else if (e.code === "ArrowLeft") {
            e.preventDefault();
            stepFrame(e.shiftKey ? -10 : -1);
        } else if (e.code === "ArrowRight") {
            e.preventDefault();
            stepFrame(e.shiftKey ? 10 : 1);
        } else if (e.code === "Home") {
            e.preventDefault();
            seekFrame(0);
        }
    });

    // Mode capteurs suiveur de ligne (Norm / Raw / Corr)
    document.querySelectorAll(".line-mode-btn").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".line-mode-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            state.lineSensorMode = btn.dataset.mode;
            updateLineChart();
        });
    });

    // Contrôles Caméra Canvas
    document.getElementById("btn-cam-follow").addEventListener("click", () => {
        state.camera.followRobot = true;
    });
    document.getElementById("btn-cam-fit").addEventListener("click", fitCameraToTrajectory);
    document.getElementById("btn-clear-trail").addEventListener("click", () => {
        // Redraw
        drawScene();
    });
    document.getElementById("btn-zoom-in").addEventListener("click", () => {
        state.camera.zoom *= 1.25;
        drawScene();
    });
    document.getElementById("btn-zoom-out").addEventListener("click", () => {
        state.camera.zoom /= 1.25;
        drawScene();
    });
    document.getElementById("btn-zoom-reset").addEventListener("click", () => {
        state.camera.zoom = 160;
        state.camera.followRobot = true;
        drawScene();
    });
}

function handleFileSelect(e) {
    if (e.target.files && e.target.files[0]) {
        parseCSVFile(e.target.files[0]);
    }
}

// ==============================================================================
// 4. PARSER CSV & EXTRACTION DES DONNÉES
// ==============================================================================
function parseCSVFile(file) {
    const reader = new FileReader();
    reader.onload = (e) => {
        const text = e.target.result;
        processCSVText(text, file.name);
    };
    reader.readAsText(file);
}

function loadSampleData() {
    fetch("sample_telemetry.csv")
        .then(response => {
            if (!response.ok) throw new Error("Fichier exemple non disponible en direct");
            return response.text();
        })
        .then(text => {
            processCSVText(text, "sample_telemetry.csv");
        })
        .catch(err => {
            console.warn("Échantillon non trouvé, génération secours...", err);
        });
}

function processCSVText(csvText, fileName = "") {
    document.getElementById("current-filename").textContent = fileName ? `(${fileName})` : "";
    
    // Détection délimiteur (, ou \t)
    const firstLine = csvText.split(/\r?\n/)[0];
    const delimiter = firstLine.includes("\t") ? "\t" : ",";

    const lines = csvText.trim().split(/\r?\n/);
    if (lines.length < 2) {
        alert("Fichier CSV invalide ou sans données !");
        return;
    }

    const headers = lines[0].split(delimiter).map(h => h.trim());
    const headerMap = {};
    headers.forEach((h, idx) => { headerMap[h] = idx; });

    const records = [];
    for (let i = 1; i < lines.length; ++i) {
        const rawLine = lines[i].trim();
        if (!rawLine) continue;
        const tokens = rawLine.split(delimiter).map(t => t.trim());
        if (tokens.length < headers.length * 0.7) continue;

        // Fonction d'extraction sécurisée
        const getVal = (name, fallback = 0) => {
            const idx = headerMap[name];
            if (idx === undefined || idx >= tokens.length) return fallback;
            const parsed = parseFloat(tokens[idx]);
            return isNaN(parsed) ? fallback : parsed;
        };

        const rec = {
            timestamp_ms: getVal("timestamp_ms"),
            seq_number: getVal("seq_number"),
            // FSM
            fsm_current_state: parseInt(getVal("fsm_current_state")),
            fsm_requested_state: parseInt(getVal("fsm_requested_state")),
            fsm_flags: parseInt(getVal("fsm_flags")),
            fsm_error_code: parseInt(getVal("fsm_error_code")),
            // Sonar
            sonar_raw_dist_cm: getVal("sonar_raw_dist_cm"),
            sonar_calc_dist_cm: getVal("sonar_calc_dist_cm"),
            sonar_flags: parseInt(getVal("sonar_flags")),
            sonar_error_code: parseInt(getVal("sonar_error_code")),
            // Line follower
            line_flags: parseInt(getVal("line_flags")),
            s: [
                { raw: getVal("s0_raw"), norm: getVal("s0_norm"), corr: getVal("s0_corr"), err: getVal("s0_err") },
                { raw: getVal("s1_raw"), norm: getVal("s1_norm"), corr: getVal("s1_corr"), err: getVal("s1_err") },
                { raw: getVal("s2_raw"), norm: getVal("s2_norm"), corr: getVal("s2_corr"), err: getVal("s2_err") },
                { raw: getVal("s3_raw"), norm: getVal("s3_norm"), corr: getVal("s3_corr"), err: getVal("s3_err") },
                { raw: getVal("s4_raw"), norm: getVal("s4_norm"), corr: getVal("s4_corr"), err: getVal("s4_err") }
            ],
            sensors_comp_flags: parseInt(getVal("sensors_comp_flags")),
            sensors_comp_err: parseInt(getVal("sensors_comp_err")),
            // Motors
            motors_state: parseInt(getVal("motors_state")),
            motors_comp_flags: parseInt(getVal("motors_comp_flags")),
            motors_error_code: parseInt(getVal("motors_error_code")),
            // Left motor
            m_left_target_throttle: getVal("m_left_target_throttle"),
            m_left_applied_throttle: getVal("m_left_applied_throttle"),
            m_left_encoder: parseInt(getVal("m_left_encoder")),
            m_left_flags: parseInt(getVal("m_left_flags")),
            m_left_pid_flags: parseInt(getVal("m_left_pid_flags")),
            m_left_kp: getVal("m_left_kp"),
            m_left_kd: getVal("m_left_kd"),
            m_left_ki: getVal("m_left_ki"),
            // Right motor
            m_right_target_throttle: getVal("m_right_target_throttle"),
            m_right_applied_throttle: getVal("m_right_applied_throttle"),
            m_right_encoder: parseInt(getVal("m_right_encoder")),
            m_right_flags: parseInt(getVal("m_right_flags")),
            m_right_pid_flags: parseInt(getVal("m_right_pid_flags")),
            m_right_kp: getVal("m_right_kp"),
            m_right_kd: getVal("m_right_kd"),
            m_right_ki: getVal("m_right_ki"),
            // Steering
            steering_target_angle: getVal("steering_target_angle"),
            steering_current_angle: getVal("steering_current_angle"),
            steering_target_ccr: parseInt(getVal("steering_target_ccr")),
            steering_current_ccr: parseInt(getVal("steering_current_ccr")),
            steering_coeff: getVal("steering_coeff"),
            steering_flags: parseInt(getVal("steering_flags")),
            steering_error_code: parseInt(getVal("steering_error_code")),
            // Aux servo
            aux_servo_target_angle: getVal("aux_servo_target_angle"),
            aux_servo_current_angle: getVal("aux_servo_current_angle"),
            aux_servo_target_ccr: parseInt(getVal("aux_servo_target_ccr")),
            aux_servo_current_ccr: parseInt(getVal("aux_servo_current_ccr")),
            // Services
            services_flags: parseInt(getVal("services_flags")),
            services_timestamp: parseInt(getVal("services_timestamp")),
            services_handler_err: parseInt(getVal("services_handler_err")),
            daemons_count: parseInt(getVal("daemons_count")),
            daemons: [
                { status: parseInt(getVal("daemon1_status")), err: parseInt(getVal("daemon1_err")) },
                { status: parseInt(getVal("daemon2_status")), err: parseInt(getVal("daemon2_err")) },
                { status: parseInt(getVal("daemon3_status")), err: parseInt(getVal("daemon3_err")) },
                { status: parseInt(getVal("daemon4_status")), err: parseInt(getVal("daemon4_err")) }
            ]
        };
        records.push(rec);
    }

    if (records.length === 0) {
        alert("Aucun enregistrement valide trouvé dans le CSV !");
        return;
    }

    state.data = records;
    state.totalFrames = records.length;
    state.currentIndex = 0;

    // 1. Calcul de la trajectoire par odométrie cinématique Ackermann
    computeOdometry();

    // 2. Calcul des KPI & métriques globales
    computeKPI();

    // 3. Mise à jour de l'UI & Scrubber
    const scrubber = document.getElementById("scrubber");
    scrubber.max = state.totalFrames - 1;
    scrubber.value = 0;
    document.getElementById("frame-total").textContent = state.totalFrames;

    // 4. Alimentation des 7 graphiques Chart.js
    populateAllCharts();

    // 5. Centrage caméra et premier rendu
    fitCameraToTrajectory();
    updateLiveHUD(0);
    drawScene();
}

// ==============================================================================
// 5. MODÉLISATION DU COMPORTEMENT DU ROBOT (ODOMÉTRIE BICYCLE / ACKERMANN)
// ==============================================================================
function computeOdometry() {
    const poses = [];
    let curX = 0.0;
    let curY = 0.0;
    let curTheta = 0.0; // En radians (0 = orienté vers +X)
    let cumDist = 0.0;

    const baseTs = state.data[0].timestamp_ms;

    for (let i = 0; i < state.data.length; ++i) {
        const d = state.data[i];
        const timeSec = (d.timestamp_ms - baseTs) / 1000.0;

        let dt = 0.05; // 20 Hz par défaut
        if (i > 0) {
            dt = (d.timestamp_ms - state.data[i - 1].timestamp_ms) / 1000.0;
            if (dt <= 0 || dt > 0.5) dt = 0.05;
        }

        // Vitesse linéaire v : estimée via encodeurs ou throttles appliqués
        let v = 0.0;
        if (i > 0) {
            const encL_prev = state.data[i - 1].m_left_encoder;
            const encR_prev = state.data[i - 1].m_right_encoder;
            const dEncL = Math.max(0, d.m_left_encoder - encL_prev);
            const dEncR = Math.max(0, d.m_right_encoder - encR_prev);
            const avgDEnc = (dEncL + dEncR) / 2.0;

            // Calibration ticks -> mètres : environ 100 ticks = 1 mètre
            if (avgDEnc > 0 && d.m_left_encoder > 0) {
                v = (avgDEnc / 100.0) / dt;
            }
        }
        
        // Si les encodeurs ne sont pas actifs/simulés, utiliser la consigne de throttle appliquée
        if (v <= 0.001) {
            const avgThrottle = (d.m_left_applied_throttle + d.m_right_applied_throttle) / 2.0;
            v = avgThrottle * MAX_VELOCITY_MPS;
        }

        // Modèle Ackermann (Bicycle Model) :
        // delta = angle de braquage des roues avant
        // dtheta/dt = (v / L) * tan(delta)
        // dx/dt = v * cos(theta)
        // dy/dt = v * sin(theta)
        const steerAngleDeg = d.steering_current_angle;
        const steerRad = (steerAngleDeg * Math.PI) / 180.0;
        
        // Vitesse angulaire
        const omega = (v / ROBOT_WHEELBASE) * Math.tan(steerRad);
        
        curTheta += omega * dt;
        const dx = v * Math.cos(curTheta) * dt;
        const dy = v * Math.sin(curTheta) * dt;
        curX += dx;
        curY += dy;
        cumDist += Math.sqrt(dx * dx + dy * dy);

        poses.push({
            x: curX,
            y: curY,
            theta: curTheta,
            v: v,
            dist: cumDist,
            timeSec: timeSec,
            steerDeg: steerAngleDeg,
            fsmState: d.fsm_current_state
        });
    }

    state.poses = poses;
}

// ==============================================================================
// 6. CALCUL DES KPI & STATISTIQUES GLOBALES
// ==============================================================================
function computeKPI() {
    if (!state.data.length) return;
    const firstTs = state.data[0].timestamp_ms;
    const lastTs = state.data[state.data.length - 1].timestamp_ms;
    const duration = (lastTs - firstTs) / 1000.0;
    const avgHz = duration > 0 ? (state.data.length / duration) : 0;
    const lastPose = state.poses[state.poses.length - 1];
    const totalDist = lastPose ? lastPose.dist : 0.0;
    
    let maxSpeed = 0.0;
    state.poses.forEach(p => { if (p.v > maxSpeed) maxSpeed = p.v; });

    let transitions = 0;
    let prevSt = null;
    let obstacleFrames = 0;
    let errorCount = 0;

    state.data.forEach(d => {
        if (prevSt !== null && d.fsm_current_state !== prevSt) {
            transitions++;
        }
        prevSt = d.fsm_current_state;

        if (d.sonar_calc_dist_cm < 25.0 && d.sonar_calc_dist_cm > 0.0) {
            obstacleFrames++;
        }

        if (d.fsm_error_code !== 0 || d.motors_error_code !== 0 || d.sensors_comp_err !== 0 || d.services_handler_err !== 0) {
            errorCount++;
        }
    });

    state.summary = {
        duration, avgHz, totalDist, maxSpeed, transitions, obstacleFrames, errorCount
    };

    document.getElementById("kpi-duration").innerHTML = `${duration.toFixed(1)} s <small style="font-size: 0.8rem; font-weight: normal; color: var(--text-muted);">(@ ${avgHz.toFixed(1)} Hz)</small>`;
    document.getElementById("kpi-distance").textContent = `${totalDist.toFixed(2)} m`;
    document.getElementById("kpi-max-speed").textContent = `${maxSpeed.toFixed(2)} m/s`;
    document.getElementById("kpi-transitions").textContent = transitions;
    document.getElementById("kpi-obstacles").textContent = `${obstacleFrames} trames`;
    
    const kpiErr = document.getElementById("kpi-errors");
    kpiErr.textContent = errorCount;
    kpiErr.style.color = errorCount > 0 ? "var(--accent-red)" : "var(--accent-green)";

    document.getElementById("total-time").textContent = formatTime(duration);
}

function exportKpiReport() {
    if (!state.data.length) return;
    const s = state.summary;
    const csvContent = "data:text/csv;charset=utf-8," +
        "Metrique,Valeur\n" +
        `Duree_totale_s,${s.duration.toFixed(2)}\n` +
        `Frequence_moyenne_Hz,${s.avgHz.toFixed(2)}\n` +
        `Distance_parcourue_m,${s.totalDist.toFixed(3)}\n` +
        `Vitesse_max_mps,${s.maxSpeed.toFixed(3)}\n` +
        `Transitions_FSM,${s.transitions}\n` +
        `Trames_obstacle_detecte,${s.obstacleFrames}\n` +
        `Trames_en_erreur,${s.errorCount}\n`;
    
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", "telemetry_kpi_summary.csv");
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

// ==============================================================================
// 7. RENDU CANVAS 2D DU ROBOT & DU PARCOURS (Thing #1)
// ==============================================================================
let canvas, ctx;

function setupCanvas() {
    canvas = document.getElementById("robot-canvas");
    ctx = canvas.getContext("2d");

    function resize() {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
        drawScene();
    }
    window.addEventListener("resize", resize);
    resize();

    // Gestion du Pan & Zoom à la souris
    canvas.addEventListener("mousedown", (e) => {
        state.camera.isDragging = true;
        state.camera.dragStartX = e.clientX;
        state.camera.dragStartY = e.clientY;
        state.camera.followRobot = false;
    });

    window.addEventListener("mousemove", (e) => {
        if (!state.camera.isDragging) return;
        const dx = e.clientX - state.camera.dragStartX;
        const dy = e.clientY - state.camera.dragStartY;
        state.camera.dragStartX = e.clientX;
        state.camera.dragStartY = e.clientY;
        state.camera.x -= dx / state.camera.zoom;
        state.camera.y += dy / state.camera.zoom;
        drawScene();
    });

    window.addEventListener("mouseup", () => {
        state.camera.isDragging = false;
    });

    canvas.addEventListener("wheel", (e) => {
        e.preventDefault();
        const factor = e.deltaY < 0 ? 1.15 : 0.87;
        state.camera.zoom *= factor;
        drawScene();
    });
}

function fitCameraToTrajectory() {
    if (!state.poses.length) return;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    state.poses.forEach(p => {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    });

    // Marge
    const pad = 0.5; // 50 cm
    minX -= pad; maxX += pad; minY -= pad; maxY += pad;

    const spanX = Math.max(0.5, maxX - minX);
    const spanY = Math.max(0.5, maxY - minY);

    state.camera.x = (minX + maxX) / 2.0;
    state.camera.y = (minY + maxY) / 2.0;

    const zoomX = canvas.width / spanX;
    const zoomY = canvas.height / spanY;
    state.camera.zoom = Math.min(zoomX, zoomY) * 0.85;
    state.camera.followRobot = false;
    drawScene();
}

// Convertit les coordonnées monde (mètres) en coordonnées Canvas (pixels)
function worldToCanvas(wx, wy) {
    const cx = canvas.width / 2 + (wx - state.camera.x) * state.camera.zoom;
    const cy = canvas.height / 2 - (wy - state.camera.y) * state.camera.zoom; // Inversion axe Y standard
    return { x: cx, y: cy };
}

function drawScene() {
    if (!canvas || !ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Suivi automatique du robot si activé
    if (state.camera.followRobot && state.poses[state.currentIndex]) {
        const curPose = state.poses[state.currentIndex];
        state.camera.x = curPose.x;
        state.camera.y = curPose.y;
    }

    // 1. Dessin de la grille monde (repères tous les 50cm et 1m)
    drawGrid();

    // 2. Dessin de la trace de la trajectoire (colorée par état FSM)
    drawTrajectoryTrail();

    // 3. Dessin du robot PiCar-S à la frame courante
    if (state.poses.length > 0 && state.poses[state.currentIndex]) {
        const curPose = state.poses[state.currentIndex];
        const curData = state.data[state.currentIndex];
        drawRobot(curPose, curData);
    }
}

function drawGrid() {
    const zoom = state.camera.zoom;
    const gridStepM = zoom > 200 ? 0.25 : (zoom > 80 ? 0.5 : 1.0); // 25cm, 50cm ou 1m
    const stepPx = gridStepM * zoom;

    const topLeft = {
        x: state.camera.x - (canvas.width / 2) / zoom,
        y: state.camera.y + (canvas.height / 2) / zoom
    };
    const bottomRight = {
        x: state.camera.x + (canvas.width / 2) / zoom,
        y: state.camera.y - (canvas.height / 2) / zoom
    };

    ctx.save();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#141c2b";

    // Lignes verticales
    const startX = Math.floor(topLeft.x / gridStepM) * gridStepM;
    for (let x = startX; x <= bottomRight.x; x += gridStepM) {
        const pt = worldToCanvas(x, 0);
        ctx.beginPath();
        ctx.moveTo(pt.x, 0);
        ctx.lineTo(pt.x, canvas.height);
        ctx.stroke();
    }

    // Lignes horizontales
    const startY = Math.floor(bottomRight.y / gridStepM) * gridStepM;
    for (let y = startY; y <= topLeft.y; y += gridStepM) {
        const pt = worldToCanvas(0, y);
        ctx.beginPath();
        ctx.moveTo(0, pt.y);
        ctx.lineTo(canvas.width, pt.y);
        ctx.stroke();
    }

    // Origine (0, 0)
    const orig = worldToCanvas(0, 0);
    ctx.strokeStyle = "#2d3f5e";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(orig.x, orig.y, 4, 0, Math.PI * 2);
    ctx.stroke();

    ctx.restore();
}

function drawTrajectoryTrail() {
    if (state.poses.length < 2) return;

    ctx.save();
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    // Trace parcourue jusqu'à l'index courant
    for (let i = 1; i <= state.currentIndex; ++i) {
        const p1 = state.poses[i - 1];
        const p2 = state.poses[i];
        const pt1 = worldToCanvas(p1.x, p1.y);
        const pt2 = worldToCanvas(p2.x, p2.y);

        const stInfo = FSM_STATES[p2.fsmState] || { color: "#3b82f6" };
        ctx.strokeStyle = stInfo.color;

        ctx.beginPath();
        ctx.moveTo(pt1.x, pt1.y);
        ctx.lineTo(pt2.x, pt2.y);
        ctx.stroke();
    }

    // Trace future (en pointillés discrets)
    if (state.currentIndex < state.poses.length - 1) {
        ctx.strokeStyle = "rgba(71, 85, 105, 0.4)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        const ptCur = worldToCanvas(state.poses[state.currentIndex].x, state.poses[state.currentIndex].y);
        ctx.moveTo(ptCur.x, ptCur.y);
        for (let i = state.currentIndex + 1; i < state.poses.length; ++i) {
            const pt = worldToCanvas(state.poses[i].x, state.poses[i].y);
            ctx.lineTo(pt.x, pt.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    ctx.restore();
}

function drawRobot(pose, data) {
    const pt = worldToCanvas(pose.x, pose.y);
    const zoom = state.camera.zoom;

    // Dimensions en pixels
    const L = ROBOT_LENGTH * zoom;
    const W = ROBOT_WIDTH * zoom;
    const wheelbase = ROBOT_WHEELBASE * zoom;
    const wheelLen = 0.045 * zoom;
    const wheelWid = 0.018 * zoom;

    ctx.save();
    ctx.translate(pt.x, pt.y);
    // Inversion de l'angle pour correspondre à l'orientation Canvas (Y descendant)
    ctx.rotate(-pose.theta);

    // ==========================================
    // A. Cône Sonar Ultrason vers l'avant
    // ==========================================
    const sonarDistM = Math.min(2.5, data.sonar_calc_dist_cm / 100.0);
    const sonarRangePx = sonarDistM * zoom;
    const fovHalfRad = (15.0 * Math.PI) / 180.0; // 30° ouverture

    const coneGrad = ctx.createRadialGradient(L * 0.5, 0, 0, L * 0.5, 0, sonarRangePx);
    let coneColor = "rgba(16, 185, 129, 0.25)"; // Vert nominal
    let arcStrokeColor = "#10b981";
    if (data.sonar_calc_dist_cm < 20.0 && data.sonar_calc_dist_cm > 0) {
        coneColor = "rgba(239, 68, 68, 0.45)"; // Rouge alerte
        arcStrokeColor = "#ef4444";
    } else if (data.sonar_calc_dist_cm < 35.0) {
        coneColor = "rgba(249, 115, 22, 0.35)"; // Orange attention
        arcStrokeColor = "#f97316";
    }

    coneGrad.addColorStop(0, "rgba(255, 255, 255, 0.05)");
    coneGrad.addColorStop(1, coneColor);

    ctx.beginPath();
    ctx.moveTo(L * 0.5, 0);
    ctx.arc(L * 0.5, 0, sonarRangePx, -fovHalfRad, fovHalfRad);
    ctx.closePath();
    ctx.fillStyle = coneGrad;
    ctx.fill();

    // Arc d'obstacle au bout du cône
    ctx.beginPath();
    ctx.arc(L * 0.5, 0, sonarRangePx, -fovHalfRad, fovHalfRad);
    ctx.lineWidth = 3;
    ctx.strokeStyle = arcStrokeColor;
    ctx.stroke();

    // ==========================================
    // B. Roues Arrières (Propulsion)
    // ==========================================
    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#475569";
    ctx.lineWidth = 1.5;

    // Roue arrière gauche
    ctx.fillRect(-wheelbase * 0.5 - wheelLen * 0.5, -W * 0.5 - wheelWid * 0.5, wheelLen, wheelWid);
    ctx.strokeRect(-wheelbase * 0.5 - wheelLen * 0.5, -W * 0.5 - wheelWid * 0.5, wheelLen, wheelWid);

    // Roue arrière droite
    ctx.fillRect(-wheelbase * 0.5 - wheelLen * 0.5, W * 0.5 - wheelWid * 0.5, wheelLen, wheelWid);
    ctx.strokeRect(-wheelbase * 0.5 - wheelLen * 0.5, W * 0.5 - wheelWid * 0.5, wheelLen, wheelWid);

    // ==========================================
    // C. Roues Avants (Directrices avec angle)
    // ==========================================
    const steerRad = (-pose.steerDeg * Math.PI) / 180.0;

    // Roue avant gauche
    ctx.save();
    ctx.translate(wheelbase * 0.5, -W * 0.5);
    ctx.rotate(steerRad);
    ctx.fillRect(-wheelLen * 0.5, -wheelWid * 0.5, wheelLen, wheelWid);
    ctx.strokeRect(-wheelLen * 0.5, -wheelWid * 0.5, wheelLen, wheelWid);
    ctx.restore();

    // Roue avant droite
    ctx.save();
    ctx.translate(wheelbase * 0.5, W * 0.5);
    ctx.rotate(steerRad);
    ctx.fillRect(-wheelLen * 0.5, -wheelWid * 0.5, wheelLen, wheelWid);
    ctx.strokeRect(-wheelLen * 0.5, -wheelWid * 0.5, wheelLen, wheelWid);
    ctx.restore();

    // ==========================================
    // D. Châssis PiCar-S
    // ==========================================
    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#06b6d4";
    ctx.lineWidth = 2;

    // Corps de la voiture avec coins arrondis
    ctx.beginPath();
    ctx.roundRect(-L * 0.45, -W * 0.42, L * 0.9, W * 0.84, 8);
    ctx.fill();
    ctx.stroke();

    // Ligne centrale de carrosserie
    ctx.strokeStyle = "rgba(6, 182, 212, 0.4)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(-L * 0.4, 0);
    ctx.lineTo(L * 0.4, 0);
    ctx.stroke();

    // ==========================================
    // E. Barre 5 Capteurs TCRT5000 (Suiveur de ligne)
    // ==========================================
    const sensorBarX = (wheelbase * 0.5 + SENSOR_BAR_OFFSET * zoom);
    const sensorSpan = W * 0.7; // Largeur du réseau de capteurs

    ctx.strokeStyle = "#334155";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sensorBarX, -sensorSpan * 0.5);
    ctx.lineTo(sensorBarX, sensorSpan * 0.5);
    ctx.stroke();

    // 5 photodiodes avec halo selon intensité normalisée (0 à 1)
    const weights = [-2, -1, 0, 1, 2];
    for (let i = 0; i < 5; ++i) {
        const sy = (weights[i] / 2.0) * (sensorSpan * 0.5);
        const normVal = Math.max(0, Math.min(1, data.s[i].norm));

        // Halo lumineux
        ctx.beginPath();
        ctx.arc(sensorBarX, sy, 3 + normVal * 4, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(6, 182, 212, ${0.2 + normVal * 0.7})`;
        ctx.fill();

        // Diode centrale
        ctx.beginPath();
        ctx.arc(sensorBarX, sy, 2.5, 0, Math.PI * 2);
        ctx.fillStyle = normVal > 0.5 ? "#ffffff" : "#0284c7";
        ctx.fill();
    }

    // ==========================================
    // F. Flèche de cap directionnelle
    // ==========================================
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.moveTo(L * 0.35, 0);
    ctx.lineTo(L * 0.22, -W * 0.15);
    ctx.lineTo(L * 0.25, 0);
    ctx.lineTo(L * 0.22, W * 0.15);
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

// ==============================================================================
// 8. CONTRÔLEUR DE REJEU TEMPOREL & HUD LIVE
// ==============================================================================
function togglePlayPause() {
    state.isPlaying = !state.isPlaying;
    const btn = document.getElementById("btn-play-pause");
    if (state.isPlaying) {
        btn.textContent = "⏸ Pause";
        btn.classList.remove("btn-success");
        btn.classList.add("btn-primary");
        state.lastFrameTime = performance.now();
        requestAnimationFrame(playbackLoop);
    } else {
        btn.textContent = "▶ Lecture";
        btn.classList.remove("btn-primary");
        btn.classList.add("btn-success");
    }
}

function playbackLoop(now) {
    if (!state.isPlaying) return;

    const dtReal = (now - state.lastFrameTime) / 1000.0;
    state.lastFrameTime = now;

    // Avancement des frames selon le sample rate du dataset et la vitesse choisie
    if (state.data.length > 1) {
        const baseDt = 0.05; // 20 Hz
        const framesToAdvance = (dtReal * state.playbackSpeed) / baseDt;
        
        let newIndex = state.currentIndex + Math.max(1, Math.round(framesToAdvance));
        if (newIndex >= state.totalFrames) {
            if (state.isLooping) {
                newIndex = 0;
            } else {
                newIndex = state.totalFrames - 1;
                togglePlayPause();
            }
        }
        seekFrame(newIndex);
    }

    if (state.isPlaying) {
        requestAnimationFrame(playbackLoop);
    }
}

function stepFrame(step) {
    const nextIdx = Math.max(0, Math.min(state.totalFrames - 1, state.currentIndex + step));
    seekFrame(nextIdx);
}

function seekFrame(index) {
    if (!state.data.length) return;
    state.currentIndex = Math.max(0, Math.min(state.totalFrames - 1, index));

    // Update Slider
    document.getElementById("scrubber").value = state.currentIndex;
    document.getElementById("frame-idx").textContent = state.currentIndex;

    // Update HUD
    updateLiveHUD(state.currentIndex);

    // Update Canvas
    drawScene();

    // Update synchronisation des curseurs graphiques
    syncChartCursor(state.currentIndex);
}

function updateLiveHUD(idx) {
    const d = state.data[idx];
    const p = state.poses[idx];
    if (!d || !p) return;

    // 1. Pose Overlay
    document.getElementById("hud-x").textContent = p.x.toFixed(2);
    document.getElementById("hud-y").textContent = p.y.toFixed(2);
    document.getElementById("hud-heading").textContent = ((p.theta * 180 / Math.PI) % 360).toFixed(1);
    document.getElementById("hud-vel").textContent = p.v.toFixed(2);
    document.getElementById("hud-dist").textContent = p.dist.toFixed(2);

    // Temps
    document.getElementById("cur-time").textContent = formatTime(p.timeSec);

    // 2. FSM & Moteurs
    const fsmInfo = FSM_STATES[d.fsm_current_state] || { name: `Unknown (${d.fsm_current_state})`, color: "#94a3b8" };
    const fsmReq = FSM_STATES[d.fsm_requested_state] || { name: "--" };
    
    const badge = document.getElementById("live-fsm-badge");
    badge.textContent = fsmInfo.name.toUpperCase();
    badge.style.backgroundColor = fsmInfo.color;

    const hudFsmName = document.getElementById("hud-fsm-name");
    hudFsmName.textContent = fsmInfo.name;
    hudFsmName.style.color = fsmInfo.color;

    document.getElementById("fsm-req-text").textContent = `Req: ${fsmReq.name}`;
    document.getElementById("hud-motors-state").textContent = MOTOR_STATES[d.motors_state] || "Unknown";
    document.getElementById("hud-fsm-flags").textContent = "0x" + d.fsm_flags.toString(16).padStart(4, "0").toUpperCase();

    // 3. Sonar Ultrason
    const sonarVal = document.getElementById("hud-sonar-val");
    sonarVal.innerHTML = `${d.sonar_calc_dist_cm.toFixed(1)} <small style="font-size: 0.8rem;">cm</small>`;
    document.getElementById("hud-sonar-raw").textContent = d.sonar_raw_dist_cm.toFixed(1);
    document.getElementById("hud-sonar-flags").textContent = "0x" + d.sonar_flags.toString(16);

    const sonarBadge = document.getElementById("sonar-badge");
    if (d.sonar_calc_dist_cm < 20.0 && d.sonar_calc_dist_cm > 0) {
        sonarBadge.textContent = "ALERTE PROCHE";
        sonarBadge.style.backgroundColor = "var(--accent-red)";
    } else if (d.sonar_calc_dist_cm < 35.0) {
        sonarBadge.textContent = "ATTENTION";
        sonarBadge.style.backgroundColor = "var(--accent-orange)";
    } else {
        sonarBadge.textContent = "DÉGAGÉ";
        sonarBadge.style.backgroundColor = "var(--accent-green)";
    }

    // 4. Moteurs Throttles & Encodeurs
    document.getElementById("hud-motor-l-val").textContent = `${(d.m_left_applied_throttle * 100).toFixed(1)}%`;
    document.getElementById("hud-motor-l-tgt").textContent = (d.m_left_target_throttle * 100).toFixed(1) + "%";
    document.getElementById("hud-motor-l-enc").textContent = d.m_left_encoder;

    document.getElementById("hud-motor-r-val").textContent = `${(d.m_right_applied_throttle * 100).toFixed(1)}%`;
    document.getElementById("hud-motor-r-tgt").textContent = (d.m_right_target_throttle * 100).toFixed(1) + "%";
    document.getElementById("hud-motor-r-enc").textContent = d.m_right_encoder;

    // 5. Servomoteur Direction
    document.getElementById("hud-steer-val").textContent = `${d.steering_current_angle.toFixed(1)}°`;
    document.getElementById("hud-steer-tgt").textContent = `${d.steering_target_angle.toFixed(1)}°`;
    document.getElementById("hud-steer-ccr").textContent = d.steering_current_ccr;

    // 6. Suiveur de ligne 5 voies
    for (let i = 0; i < 5; ++i) {
        const fill = document.getElementById(`sfill-${i}`);
        const pct = Math.max(0, Math.min(100, d.s[i].norm * 100));
        fill.style.width = `${pct}%`;
    }
    document.getElementById("hud-line-err").textContent = d.sensors_comp_err;

    // 7. Bannière Santé
    const healthBanner = document.getElementById("health-banner");
    const healthText = document.getElementById("health-text");
    const healthCodes = document.getElementById("health-codes");
    const totalErr = d.fsm_error_code | d.motors_error_code | d.sensors_comp_err | d.services_handler_err;

    if (totalErr === 0) {
        healthBanner.style.background = "rgba(16, 185, 129, 0.15)";
        healthBanner.style.borderColor = "var(--accent-green)";
        healthText.textContent = "✓ Système Nominal — Zéro anomalie active";
        healthCodes.textContent = "Codes: 0x00";
    } else {
        healthBanner.style.background = "rgba(239, 68, 68, 0.2)";
        healthBanner.style.borderColor = "var(--accent-red)";
        healthText.textContent = "⚠ Anomalie active détectée !";
        healthCodes.textContent = `FSM: 0x${d.fsm_error_code.toString(16)} | Mot: 0x${d.motors_error_code.toString(16)} | Sens: 0x${d.sensors_comp_err.toString(16)}`;
    }
}

function formatTime(seconds) {
    const m = Math.floor(seconds / 60);
    const s = Math.floor(seconds % 60);
    const ms = Math.floor((seconds % 1) * 1000);
    return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}.${ms.toString().padStart(3, '0')}`;
}

// ==============================================================================
// 9. GRAPHIQUES DÉTAILLÉS SYNCHRONISÉS (Thing #2 - Chart.js)
// ==============================================================================
const chartTheme = {
    textColor: "#94a3b8",
    gridColor: "#1a2538",
    fontFamily: "'Inter', sans-serif"
};

function initCharts() {
    if (typeof Chart === "undefined") {
        console.warn("Chart.js non encore chargé.");
        return;
    }

    Chart.defaults.color = chartTheme.textColor;
    Chart.defaults.font.family = chartTheme.fontFamily;
    Chart.defaults.animation = false;
    Chart.defaults.responsive = true;
    Chart.defaults.maintainAspectRatio = false;

    // 1. Chart FSM & Motors States
    state.charts.fsm = new Chart(document.getElementById("chart-fsm"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("État Numérique", {
            stepped: true,
            ticksCallback: (val) => FSM_STATES[val]?.name || val
        })
    });

    // 2. Chart Suiveur de Ligne (5 Voies)
    state.charts.line = new Chart(document.getElementById("chart-line"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Niveau Capteur")
    });

    // 3. Chart Sonar Ultrason
    state.charts.sonar = new Chart(document.getElementById("chart-sonar"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Distance (cm)")
    });

    // 4. Chart Moteurs Throttles & Encoders
    state.charts.motors = new Chart(document.getElementById("chart-motors"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Throttle / Vitesse")
    });

    // 5. Chart Direction & Servos
    state.charts.steering = new Chart(document.getElementById("chart-steering"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Angle (°)")
    });

    // 6. Chart Paramètres PID
    state.charts.pid = new Chart(document.getElementById("chart-pid"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Gain")
    });

    // 7. Chart Diagnostic Daemons & Erreurs
    state.charts.diag = new Chart(document.getElementById("chart-diag"), {
        type: "line",
        data: { labels: [], datasets: [] },
        options: createChartOptions("Status / Code")
    });

    // Ajout d'écouteurs de clic sur chaque canvas de graphe pour sauter au timestamp
    Object.values(state.charts).forEach(c => {
        c.canvas.addEventListener("click", (e) => {
            const elements = c.getElementsAtEventForMode(e, "index", { intersect: false }, true);
            if (elements.length > 0) {
                seekFrame(elements[0].index);
            }
        });
    });
}

function createChartOptions(yAxisTitle = "", extra = {}) {
    return {
        interaction: {
            mode: "index",
            intersect: false
        },
        plugins: {
            legend: {
                position: "top",
                labels: { boxWidth: 12, font: { size: 11 } }
            },
            tooltip: {
                backgroundColor: "rgba(10, 14, 23, 0.9)",
                borderColor: "#26354a",
                borderWidth: 1,
                callbacks: extra.tooltipCallbacks || {}
            }
        },
        scales: {
            x: {
                grid: { color: chartTheme.gridColor },
                ticks: { maxTicksLimit: 12, color: chartTheme.textColor }
            },
            y: {
                title: { display: !!yAxisTitle, text: yAxisTitle, color: chartTheme.textColor },
                grid: { color: chartTheme.gridColor },
                ticks: {
                    color: chartTheme.textColor,
                    callback: extra.ticksCallback || (v => v)
                }
            }
        }
    };
}

function populateAllCharts() {
    if (!state.data.length || typeof Chart === "undefined") return;

    const baseTs = state.data[0].timestamp_ms;
    const labels = state.data.map(d => ((d.timestamp_ms - baseTs) / 1000.0).toFixed(2) + "s");

    // 1. FSM & Motors States
    state.charts.fsm.data = {
        labels: labels,
        datasets: [
            {
                label: "FSM État Courant",
                data: state.data.map(d => d.fsm_current_state),
                borderColor: "#10b981",
                stepped: true,
                borderWidth: 2.5,
                pointRadius: 0
            },
            {
                label: "FSM État Requis",
                data: state.data.map(d => d.fsm_requested_state),
                borderColor: "#06b6d4",
                borderDash: [5, 5],
                stepped: true,
                borderWidth: 1.5,
                pointRadius: 0
            },
            {
                label: "État Moteurs (0=Init, 1=Idle, 2=Run)",
                data: state.data.map(d => d.motors_state),
                borderColor: "#f59e0b",
                stepped: true,
                borderWidth: 1.5,
                pointRadius: 0
            }
        ]
    };
    state.charts.fsm.update();

    // 2. Line Follower
    updateLineChart(labels);

    // 3. Sonar Ultrason
    state.charts.sonar.data = {
        labels: labels,
        datasets: [
            {
                label: "Distance Filtrée EMA (cm)",
                data: state.data.map(d => d.sonar_calc_dist_cm),
                borderColor: "#06b6d4",
                backgroundColor: "rgba(6, 182, 212, 0.1)",
                fill: true,
                borderWidth: 2,
                pointRadius: 0
            },
            {
                label: "Distance Brute (cm)",
                data: state.data.map(d => d.sonar_raw_dist_cm),
                borderColor: "#64748b",
                borderWidth: 1,
                pointRadius: 0
            },
            {
                label: "Seuil Alerte (20 cm)",
                data: state.data.map(() => 20.0),
                borderColor: "#ef4444",
                borderDash: [4, 4],
                borderWidth: 1.5,
                pointRadius: 0
            }
        ]
    };
    state.charts.sonar.update();

    // 4. Moteurs Throttles
    state.charts.motors.data = {
        labels: labels,
        datasets: [
            {
                label: "Gauche Appliqué",
                data: state.data.map(d => d.m_left_applied_throttle),
                borderColor: "#10b981",
                borderWidth: 2,
                pointRadius: 0
            },
            {
                label: "Gauche Cible",
                data: state.data.map(d => d.m_left_target_throttle),
                borderColor: "#34d399",
                borderDash: [4, 4],
                borderWidth: 1.5,
                pointRadius: 0
            },
            {
                label: "Droit Appliqué",
                data: state.data.map(d => d.m_right_applied_throttle),
                borderColor: "#3b82f6",
                borderWidth: 2,
                pointRadius: 0
            },
            {
                label: "Droit Cible",
                data: state.data.map(d => d.m_right_target_throttle),
                borderColor: "#60a5fa",
                borderDash: [4, 4],
                borderWidth: 1.5,
                pointRadius: 0
            }
        ]
    };
    state.charts.motors.update();

    // 5. Servomoteur Direction
    state.charts.steering.data = {
        labels: labels,
        datasets: [
            {
                label: "Angle Actuel (°)",
                data: state.data.map(d => d.steering_current_angle),
                borderColor: "#f59e0b",
                borderWidth: 2,
                pointRadius: 0
            },
            {
                label: "Angle Cible (°)",
                data: state.data.map(d => d.steering_target_angle),
                borderColor: "#fbbf24",
                borderDash: [4, 4],
                borderWidth: 1.5,
                pointRadius: 0
            }
        ]
    };
    state.charts.steering.update();

    // 6. Paramètres PID
    state.charts.pid.data = {
        labels: labels,
        datasets: [
            { label: "Left Kp", data: state.data.map(d => d.m_left_kp), borderColor: "#ec4899", borderWidth: 1.5, pointRadius: 0 },
            { label: "Left Kd", data: state.data.map(d => d.m_left_kd), borderColor: "#f472b6", borderWidth: 1.5, pointRadius: 0 },
            { label: "Left Ki", data: state.data.map(d => d.m_left_ki), borderColor: "#fbcfe8", borderWidth: 1.5, pointRadius: 0 }
        ]
    };
    state.charts.pid.update();

    // 7. Supervision Daemons & Diagnostics
    state.charts.diag.data = {
        labels: labels,
        datasets: [
            { label: "Daemon 1 (Status)", data: state.data.map(d => d.daemons[0]?.status ?? 0), borderColor: "#06b6d4", stepped: true, borderWidth: 1.5, pointRadius: 0 },
            { label: "Daemon 2 (Status)", data: state.data.map(d => d.daemons[1]?.status ?? 0), borderColor: "#10b981", stepped: true, borderWidth: 1.5, pointRadius: 0 },
            { label: "Code Erreur FSM", data: state.data.map(d => d.fsm_error_code), borderColor: "#ef4444", borderWidth: 2, pointRadius: 0 },
            { label: "Code Erreur Moteurs", data: state.data.map(d => d.motors_error_code), borderColor: "#dc2626", borderWidth: 2, pointRadius: 0 }
        ]
    };
    state.charts.diag.update();
}

function updateLineChart(labels = null) {
    if (!state.charts.line || !state.data.length) return;
    if (!labels) {
        const baseTs = state.data[0].timestamp_ms;
        labels = state.data.map(d => ((d.timestamp_ms - baseTs) / 1000.0).toFixed(2) + "s");
    }

    const mode = state.lineSensorMode; // "norm", "raw", "corr"
    const colors = ["#ef4444", "#f59e0b", "#10b981", "#3b82f6", "#8b5cf6"];
    const names = ["S0 (Ext G)", "S1 (Mid G)", "S2 (Centre)", "S3 (Mid D)", "S4 (Ext D)"];

    state.charts.line.data = {
        labels: labels,
        datasets: [0, 1, 2, 3, 4].map(i => ({
            label: `${names[i]} [${mode}]`,
            data: state.data.map(d => d.s[i][mode]),
            borderColor: colors[i],
            borderWidth: 1.8,
            pointRadius: 0
        }))
    };
    state.charts.line.update();
}

// Synchronise le curseur d'inspection sur les graphiques
function syncChartCursor(index) {
    // Si Chart.js supporte active elements, on peut mettre en évidence la tranche temporelle
    Object.values(state.charts).forEach(c => {
        if (!c.data || !c.data.datasets || !c.data.datasets.length) return;
        c.setActiveElements([{ datasetIndex: 0, index: index }]);
        c.tooltip.setActiveElements([{ datasetIndex: 0, index: index }], { x: 0, y: 0 });
        c.update('none');
    });
}
