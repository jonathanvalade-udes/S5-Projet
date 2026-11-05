// ==============================================================================
// FastDrink Telemetry Client & Dashboard Frontend (JS)
// Découplage complet Réseau / Rendu (requestAnimationFrame + DOM Cache)
// ==============================================================================

let eventSource = null;
let packetCount = 0;
let frameCount = 0;
let rateInterval = null;

let latestTelemetry = null;
let latestSeq = null;
let prevSeq = null;
let newTelemetryAvailable = false;

let latestStatus = null;
let newStatusAvailable = false;

// Cache statique des éléments DOM pour éviter tout layout thrashing
const DOM = {};
const lastRendered = {};

// Initialisation au chargement du DOM
window.addEventListener('DOMContentLoaded', () => {
    initDOMCache();
    initEventSource();
    startRateCalculator();
    requestAnimationFrame(renderLoop);
});

function initDOMCache() {
    DOM.connectionBadge = document.getElementById('connection-status');
    DOM.connectionText = document.getElementById('connection-text');
    DOM.targetAddress = document.getElementById('target-address');
    DOM.packetSeq = document.getElementById('packet-seq');
    DOM.packetRate = document.getElementById('packet-rate');

    DOM.fsmBadge = document.getElementById('fsm-badge');
    DOM.fsmId = document.getElementById('fsm-id');

    DOM.sonarDist = document.getElementById('sonar-dist');
    DOM.sonarRaw = document.getElementById('sonar-raw');
    DOM.sonarProgress = document.getElementById('sonar-progress');
    DOM.sonarZone = document.getElementById('sonar-zone');

    DOM.lineStatus = document.getElementById('line-status');
    DOM.lineNodes = [];
    DOM.lineVals = [];
    for (let i = 0; i < 5; ++i) {
        DOM.lineNodes.push(document.getElementById(`sensor-${i}`));
        DOM.lineVals.push(document.getElementById(`sval-${i}`));
    }

    DOM.motorLeftApp = document.getElementById('motor-left-app');
    DOM.motorLeftTgt = document.getElementById('motor-left-tgt');
    DOM.motorLeftFill = document.getElementById('motor-left-fill');

    DOM.motorRightApp = document.getElementById('motor-right-app');
    DOM.motorRightTgt = document.getElementById('motor-right-tgt');
    DOM.motorRightFill = document.getElementById('motor-right-fill');

    DOM.steeringVal = document.getElementById('steering-val');
    DOM.steeringInd = document.getElementById('steering-indicator');

    DOM.cmdLogs = document.getElementById('cmd-logs');
}

// ------------------------------------------------------------------------------
// Couche Réseau SSE : zéro opération DOM, uniquement stockage de la dernière trame
// ------------------------------------------------------------------------------
function initEventSource() {
    if (eventSource) {
        eventSource.close();
    }

    eventSource = new EventSource('/api/stream');

    eventSource.onopen = () => {
        if (DOM.connectionText && DOM.connectionText.textContent === 'Serveur Web Déconnecté') {
            if (DOM.connectionBadge) DOM.connectionBadge.className = 'status-badge disconnected';
            DOM.connectionText.textContent = 'En attente du robot...';
        }
    };

    eventSource.addEventListener('status', (e) => {
        try {
            latestStatus = JSON.parse(e.data);
            newStatusAvailable = true;
        } catch (err) {
            console.error('Erreur parsing status:', err);
        }
    });

    eventSource.addEventListener('telemetry', (e) => {
        try {
            latestTelemetry = JSON.parse(e.data);
            if (latestTelemetry && latestTelemetry.seq !== undefined) {
                latestSeq = latestTelemetry.seq;
            }
            packetCount++;
            newTelemetryAvailable = true;
        } catch (err) {
            console.error('Erreur parsing telemetry:', err);
        }
    });

    eventSource.onerror = () => {
        if (DOM.connectionBadge && DOM.connectionText) {
            DOM.connectionBadge.className = 'status-badge disconnected';
            DOM.connectionText.textContent = 'Serveur Web Déconnecté';
        }
    };
}

// ------------------------------------------------------------------------------
// Boucle de Rendu cadencée par requestAnimationFrame (60 FPS)
// ------------------------------------------------------------------------------
function renderLoop() {
    if (newStatusAvailable && latestStatus) {
        newStatusAvailable = false;
        applyStatus(latestStatus);
    }

    if (newTelemetryAvailable && latestTelemetry) {
        newTelemetryAvailable = false;
        applyTelemetry(latestTelemetry);
        frameCount++;
    }

    requestAnimationFrame(renderLoop);
}

// Helpers de dirty-checking : modification DOM UNIQUEMENT si la valeur a changé
function updateText(el, key, val) {
    if (el && lastRendered[key] !== val) {
        el.textContent = val;
        lastRendered[key] = val;
    }
}

function updateWidth(el, key, pctStr) {
    if (el && lastRendered[key] !== pctStr) {
        el.style.width = pctStr;
        lastRendered[key] = pctStr;
    }
}

function updateLeft(el, key, pctStr) {
    if (el && lastRendered[key] !== pctStr) {
        el.style.left = pctStr;
        lastRendered[key] = pctStr;
    }
}

function updateActive(el, key, isActive) {
    if (el && lastRendered[key] !== isActive) {
        if (isActive) el.classList.add('active');
        else el.classList.remove('active');
        lastRendered[key] = isActive;
    }
}

function valNum(v, fallback = 0) {
    return (typeof v === 'number' && !isNaN(v) && isFinite(v)) ? v : fallback;
}

// ------------------------------------------------------------------------------
// Application des données au DOM
// ------------------------------------------------------------------------------
function applyStatus(status) {
    if (!status) return;

    if (status.ip && status.port) {
        updateText(DOM.targetAddress, 'target_addr', `${status.ip}:${status.port}`);
    }

    const isConnected = !!status.connected;
    if (lastRendered['connected'] !== isConnected) {
        lastRendered['connected'] = isConnected;
        if (DOM.connectionBadge && DOM.connectionText) {
            if (isConnected) {
                DOM.connectionBadge.className = 'status-badge connected';
                DOM.connectionText.textContent = 'Robot Connecté';
            } else {
                DOM.connectionBadge.className = 'status-badge disconnected';
                DOM.connectionText.textContent = 'En recherche du robot...';
            }
        }
    }
}

function applyTelemetry(telem) {
    if (!telem || typeof telem !== 'object') return;

    // 1. Seq number
    if (telem.seq !== undefined) {
        updateText(DOM.packetSeq, 'seq', telem.seq);
    }

    // 2. FSM State
    if (telem.fsm) {
        const stateName = telem.fsm.state_name || 'Inconnu';
        const stateId = telem.fsm.state_id !== undefined ? telem.fsm.state_id : '--';
        updateText(DOM.fsmBadge, 'fsm_name', stateName);
        updateText(DOM.fsmId, 'fsm_id', stateId);

        if (lastRendered['fsm_style'] !== stateName) {
            lastRendered['fsm_style'] = stateName;
            const lower = stateName.toLowerCase();
            if (DOM.fsmBadge) {
                if (lower.includes('stop') || lower.includes('collision')) {
                    DOM.fsmBadge.style.color = 'var(--accent-red)';
                    DOM.fsmBadge.style.borderColor = 'rgba(239, 68, 68, 0.4)';
                    DOM.fsmBadge.style.backgroundColor = 'rgba(239, 68, 68, 0.15)';
                } else if (lower.includes('line') || lower.includes('armed')) {
                    DOM.fsmBadge.style.color = 'var(--accent-green)';
                    DOM.fsmBadge.style.borderColor = 'rgba(34, 197, 94, 0.4)';
                    DOM.fsmBadge.style.backgroundColor = 'rgba(34, 197, 94, 0.15)';
                } else {
                    DOM.fsmBadge.style.color = 'var(--accent-blue)';
                    DOM.fsmBadge.style.borderColor = 'rgba(56, 189, 248, 0.4)';
                    DOM.fsmBadge.style.backgroundColor = 'rgba(56, 189, 248, 0.15)';
                }
            }
        }
    }

    // 3. Sonar Distance
    if (telem.sensors && telem.sensors.ultrasonic) {
        const u = telem.sensors.ultrasonic;
        const dist = valNum(u.calc_dist_cm, 0);
        const raw = valNum(u.raw_dist_cm, 0);

        updateText(DOM.sonarDist, 's_dist', dist.toFixed(1));
        updateText(DOM.sonarRaw, 's_raw', raw.toFixed(1));

        const pct = Math.min(100, Math.max(0, (dist / 100.0) * 100));
        updateWidth(DOM.sonarProgress, 's_prog', `${pct.toFixed(0)}%`);

        let zoneText = 'Dégagé';
        let zoneColor = 'var(--accent-green)';
        if (dist <= 15) {
            zoneText = 'CRITIQUE / STOP';
            zoneColor = 'var(--accent-red)';
        } else if (dist <= 30) {
            zoneText = 'ALERTE';
            zoneColor = 'var(--accent-yellow)';
        }
        updateText(DOM.sonarZone, 's_zone', zoneText);
        if (DOM.sonarZone && lastRendered['s_zone_col'] !== zoneColor) {
            DOM.sonarZone.style.color = zoneColor;
            lastRendered['s_zone_col'] = zoneColor;
        }
    }

    // 4. Line Follower Sensors (5x)
    if (telem.sensors && telem.sensors.line_follower && Array.isArray(telem.sensors.line_follower.sensors)) {
        const sensors = telem.sensors.line_follower.sensors;
        let anyDetected = false;
        for (let i = 0; i < sensors.length && i < 5; ++i) {
            const s = sensors[i] || {};
            const val = valNum(s.normalized, 0);
            const rawVal = valNum(s.raw, 0);

            updateText(DOM.lineVals[i], `sval_${i}`, val.toFixed(2));
            const isActive = val >= 0.5 || rawVal > 500;
            updateActive(DOM.lineNodes[i], `snode_${i}`, isActive);
            if (isActive) anyDetected = true;
        }

        const lineText = anyDetected ? 'Ligne Détectée' : 'Ligne Perdue / Cherche';
        const lineColor = anyDetected ? 'var(--accent-green)' : 'var(--accent-yellow)';
        updateText(DOM.lineStatus, 'line_status', lineText);
        if (DOM.lineStatus && lastRendered['line_col'] !== lineColor) {
            DOM.lineStatus.style.color = lineColor;
            lastRendered['line_col'] = lineColor;
        }
    }

    // 5. Motors & Steering
    if (telem.motors) {
        if (telem.motors.left) {
            const l = telem.motors.left;
            const appliedPct = Math.round(valNum(l.applied_throttle, 0) * 100);
            const targetPct = Math.round(valNum(l.target_throttle, 0) * 100);
            updateText(DOM.motorLeftApp, 'm_l_app', `${appliedPct}%`);
            updateText(DOM.motorLeftTgt, 'm_l_tgt', `${targetPct}%`);
            updateWidth(DOM.motorLeftFill, 'm_l_fill', `${Math.min(100, Math.max(0, appliedPct))}%`);
        }
        if (telem.motors.right) {
            const r = telem.motors.right;
            const appliedPct = Math.round(valNum(r.applied_throttle, 0) * 100);
            const targetPct = Math.round(valNum(r.target_throttle, 0) * 100);
            updateText(DOM.motorRightApp, 'm_r_app', `${appliedPct}%`);
            updateText(DOM.motorRightTgt, 'm_r_tgt', `${targetPct}%`);
            updateWidth(DOM.motorRightFill, 'm_r_fill', `${Math.min(100, Math.max(0, appliedPct))}%`);
        }
        if (telem.motors.steering) {
            const st = telem.motors.steering;
            const angle = valNum(st.current_angle, 0);
            updateText(DOM.steeringVal, 'st_val', `${angle.toFixed(1)}°`);
            const pointerPct = Math.min(100, Math.max(0, ((angle + 45.0) / 90.0) * 100));
            updateLeft(DOM.steeringInd, 'st_ind', `${pointerPct.toFixed(1)}%`);
        }
    }
}

// ------------------------------------------------------------------------------
// Mesure du débit réel entrant
// ------------------------------------------------------------------------------
function startRateCalculator() {
    if (rateInterval) clearInterval(rateInterval);
    rateInterval = setInterval(() => {
        let robotHz = 0;
        if (latestSeq !== null && prevSeq !== null) {
            robotHz = (latestSeq - prevSeq + 65536) % 65536;
        } else {
            robotHz = packetCount;
        }
        prevSeq = latestSeq;
        packetCount = 0;

        const currentFps = frameCount;
        frameCount = 0;

        if (DOM.packetRate) {
            if (robotHz > 0) {
                DOM.packetRate.textContent = `${robotHz} Hz (${currentFps} FPS)`;
            } else {
                DOM.packetRate.textContent = `0 Hz`;
            }
        }
    }, 1000);
}

// ------------------------------------------------------------------------------
// Commandes et console utilisateur
// ------------------------------------------------------------------------------
async function sendCommand(cmdStr) {
    if (!cmdStr || cmdStr.trim().length === 0) return;

    logConsole(`[TX] Envoi : "${cmdStr}"`, 'sent');

    try {
        const resp = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'text/plain' },
            body: cmdStr
        });

        if (resp.ok) {
            const resJson = await resp.json();
            logConsole(`[RX] Ack serveur : ${resJson.status}`, 'received');
        } else {
            logConsole(`[ERR] HTTP ${resp.status} lors de l'envoi`, 'error');
        }
    } catch (err) {
        logConsole(`[ERR] Échec de transmission : ${err.message}`, 'error');
    }
}

function applyParam(paramName, inputId) {
    const input = document.getElementById(inputId);
    if (!input) return;
    const val = input.value;
    const cmd = `${paramName}=${val}`;
    sendCommand(cmd);
}

function submitCustomCommand() {
    const input = document.getElementById('custom-cmd-input');
    if (!input) return;
    const cmd = input.value.trim();
    if (cmd) {
        sendCommand(cmd);
        input.value = '';
    }
}

function logConsole(message, type = 'system') {
    if (!DOM.cmdLogs) return;
    const time = new Date().toLocaleTimeString();

    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `[${time}] ${message}`;

    DOM.cmdLogs.appendChild(entry);
    DOM.cmdLogs.scrollTop = DOM.cmdLogs.scrollHeight;
}
