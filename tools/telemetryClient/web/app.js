// ==============================================================================
// FastDrink Telemetry Client & Dashboard Frontend (JS)
// ==============================================================================

let eventSource = null;
let lastPacketTime = Date.now();
let packetCount = 0;
let rateInterval = null;

// Initialisation
window.addEventListener('DOMContentLoaded', () => {
    initEventSource();
    startRateCalculator();
});

function initEventSource() {
    if (eventSource) {
        eventSource.close();
    }

    eventSource = new EventSource('/api/stream');

    eventSource.addEventListener('status', (e) => {
        try {
            const data = JSON.parse(e.data);
            updateStatus(data);
        } catch (err) {
            console.error('Erreur parsing status:', err);
        }
    });

    eventSource.addEventListener('telemetry', (e) => {
        try {
            const data = JSON.parse(e.data);
            packetCount++;
            updateTelemetry(data);
        } catch (err) {
            console.error('Erreur parsing telemetry:', err);
        }
    });

    eventSource.onerror = (err) => {
        const badge = document.getElementById('connection-status');
        const text = document.getElementById('connection-text');
        badge.className = 'status-badge disconnected';
        text.textContent = 'Serveur Web Déconnecté';
    };
}

function startRateCalculator() {
    if (rateInterval) clearInterval(rateInterval);
    rateInterval = setInterval(() => {
        const rateEl = document.getElementById('packet-rate');
        rateEl.textContent = `${packetCount} Hz`;
        packetCount = 0;
    }, 1000);
}

function updateStatus(status) {
    const badge = document.getElementById('connection-status');
    const text = document.getElementById('connection-text');
    const target = document.getElementById('target-address');

    if (status.ip && status.port) {
        target.textContent = `${status.ip}:${status.port}`;
    }

    if (status.connected) {
        badge.className = 'status-badge connected';
        text.textContent = 'Robot Connecté';
    } else {
        badge.className = 'status-badge disconnected';
        text.textContent = 'En recherche du robot...';
    }
}

function updateTelemetry(telem) {
    // 1. Seq number
    if (telem.seq !== undefined) {
        document.getElementById('packet-seq').textContent = telem.seq;
    }

    // 2. FSM State
    if (telem.fsm) {
        const badge = document.getElementById('fsm-badge');
        const idText = document.getElementById('fsm-id');
        badge.textContent = telem.fsm.state_name || 'Inconnu';
        idText.textContent = telem.fsm.state_id !== undefined ? telem.fsm.state_id : '--';

        // Styling badge selon état
        const stateName = (telem.fsm.state_name || '').toLowerCase();
        if (stateName.includes('stop') || stateName.includes('collision')) {
            badge.style.color = 'var(--accent-red)';
            badge.style.borderColor = 'rgba(239, 68, 68, 0.4)';
            badge.style.backgroundColor = 'rgba(239, 68, 68, 0.15)';
        } else if (stateName.includes('line') || stateName.includes('armed')) {
            badge.style.color = 'var(--accent-green)';
            badge.style.borderColor = 'rgba(34, 197, 94, 0.4)';
            badge.style.backgroundColor = 'rgba(34, 197, 94, 0.15)';
        } else {
            badge.style.color = 'var(--accent-blue)';
            badge.style.borderColor = 'rgba(56, 189, 248, 0.4)';
            badge.style.backgroundColor = 'rgba(56, 189, 248, 0.15)';
        }
    }

    // 3. Sonar Distance
    if (telem.sensors && telem.sensors.ultrasonic) {
        const u = telem.sensors.ultrasonic;
        const dist = u.calc_dist_cm !== undefined ? u.calc_dist_cm : 0;
        const raw = u.raw_dist_cm !== undefined ? u.raw_dist_cm : 0;

        document.getElementById('sonar-dist').textContent = dist.toFixed(1);
        document.getElementById('sonar-raw').textContent = raw.toFixed(1);

        const pct = Math.min(100, Math.max(0, (dist / 100.0) * 100));
        document.getElementById('sonar-progress').style.width = `${pct}%`;

        const zone = document.getElementById('sonar-zone');
        if (dist <= 15) {
            zone.textContent = 'CRITIQUE / STOP';
            zone.style.color = 'var(--accent-red)';
        } else if (dist <= 30) {
            zone.textContent = 'ALERTE';
            zone.style.color = 'var(--accent-yellow)';
        } else {
            zone.textContent = 'Dégagé';
            zone.style.color = 'var(--accent-green)';
        }
    }

    // 4. Line Follower Sensors (5x)
    if (telem.sensors && telem.sensors.line_follower && telem.sensors.line_follower.sensors) {
        const sensors = telem.sensors.line_follower.sensors;
        let anyDetected = false;
        for (let i = 0; i < sensors.length && i < 5; ++i) {
            const s = sensors[i];
            const node = document.getElementById(`sensor-${i}`);
            const valEl = document.getElementById(`sval-${i}`);

            const val = s.normalized !== undefined ? s.normalized : 0;
            valEl.textContent = val.toFixed(2);

            // Détection de ligne noire (seuil normalisé >= 0.5)
            const isActive = val >= 0.5 || (s.raw > 500);
            if (isActive) {
                node.classList.add('active');
                anyDetected = true;
            } else {
                node.classList.remove('active');
            }
        }

        const lineStatus = document.getElementById('line-status');
        lineStatus.textContent = anyDetected ? 'Ligne Détectée' : 'Ligne Perdue / Cherche';
        lineStatus.style.color = anyDetected ? 'var(--accent-green)' : 'var(--accent-yellow)';
    }

    // 5. Motors & Steering
    if (telem.motors) {
        if (telem.motors.left) {
            const l = telem.motors.left;
            const appliedPct = Math.round((l.applied_throttle || 0) * 100);
            const targetPct = Math.round((l.target_throttle || 0) * 100);
            document.getElementById('motor-left-app').textContent = `${appliedPct}%`;
            document.getElementById('motor-left-tgt').textContent = `${targetPct}%`;
            document.getElementById('motor-left-fill').style.width = `${Math.min(100, Math.max(0, appliedPct))}%`;
        }
        if (telem.motors.right) {
            const r = telem.motors.right;
            const appliedPct = Math.round((r.applied_throttle || 0) * 100);
            const targetPct = Math.round((r.target_throttle || 0) * 100);
            document.getElementById('motor-right-app').textContent = `${appliedPct}%`;
            document.getElementById('motor-right-tgt').textContent = `${targetPct}%`;
            document.getElementById('motor-right-fill').style.width = `${Math.min(100, Math.max(0, appliedPct))}%`;
        }
        if (telem.motors.steering) {
            const st = telem.motors.steering;
            const angle = st.current_angle !== undefined ? st.current_angle : 0;
            document.getElementById('steering-val').textContent = `${angle.toFixed(1)}°`;

            // Normalisation de [-45°, +45°] vers [0%, 100%]
            const pointerPct = Math.min(100, Math.max(0, ((angle + 45.0) / 90.0) * 100));
            document.getElementById('steering-indicator').style.left = `${pointerPct}%`;
        }
    }
}

// Envoi d'une commande arbitraire
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

// Application d'un paramètre clé=valeur
function applyParam(paramName, inputId) {
    const input = document.getElementById(inputId);
    if (!input) return;
    const val = input.value;
    const cmd = `${paramName}=${val}`;
    sendCommand(cmd);
}

// Soumission du formulaire console
function submitCustomCommand() {
    const input = document.getElementById('custom-cmd-input');
    const cmd = input.value.trim();
    if (cmd) {
        sendCommand(cmd);
        input.value = '';
    }
}

// Journalisation dans la console intégrée
function logConsole(message, type = 'system') {
    const logs = document.getElementById('cmd-logs');
    const time = new Date().toLocaleTimeString();

    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `[${time}] ${message}`;

    logs.appendChild(entry);
    logs.scrollTop = logs.scrollHeight;
}
