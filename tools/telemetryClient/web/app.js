// ==============================================================================
// FastDrink Telemetry Client & Dashboard Frontend (JS)
// Découplage complet Réseau / Rendu (requestAnimationFrame + DOM Cache)
// Support Diagnostic Exhaustif : Codes Erreurs, Registres PWM/ADC, Drapeaux, Daemons
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
    buildParamPanel();
    initEventSource();
    startRateCalculator();
    requestAnimationFrame(renderLoop);
});

function initDOMCache() {
    // Statut Connexion & Trames
    DOM.connectionBadge = document.getElementById('connection-status');
    DOM.connectionText = document.getElementById('connection-text');
    DOM.targetAddress = document.getElementById('target-address');
    DOM.packetSeq = document.getElementById('packet-seq');
    DOM.packetRate = document.getElementById('packet-rate');

    // Diagnostic & Erreurs
    DOM.healthBanner = document.getElementById('system-health-banner');
    DOM.healthStatusText = document.getElementById('health-status-text');
    DOM.errFsm = document.getElementById('err-fsm');
    DOM.errMotors = document.getElementById('err-motors');
    DOM.errSteering = document.getElementById('err-steering');
    DOM.errAuxServo = document.getElementById('err-aux-servo');
    DOM.errSensors = document.getElementById('err-sensors');
    DOM.errSonar = document.getElementById('err-sonar');
    DOM.errHandler = document.getElementById('err-handler');

    // Daemons
    DOM.daemons = [];
    for (let i = 1; i <= 4; ++i) {
        DOM.daemons.push({
            container: document.getElementById(`daemon-${i}`),
            status: document.getElementById(`d${i}-status`),
            err: document.getElementById(`d${i}-err`)
        });
    }
    DOM.servicesTimestamp = document.getElementById('services-timestamp');
    DOM.servicesDaemonsCount = document.getElementById('services-daemons-count');

    // FSM
    DOM.fsmBadge = document.getElementById('fsm-badge');
    DOM.fsmId = document.getElementById('fsm-id');
    DOM.fsmReqBadge = document.getElementById('fsm-req-badge');
    DOM.fsmReqId = document.getElementById('fsm-req-id');
    DOM.fsmFlagsHex = document.getElementById('fsm-flags-hex');

    // Sonar
    DOM.sonarDist = document.getElementById('sonar-dist');
    DOM.sonarRaw = document.getElementById('sonar-raw');
    DOM.sonarProgress = document.getElementById('sonar-progress');
    DOM.sonarZone = document.getElementById('sonar-zone');
    DOM.sonarFlagsRaw = document.getElementById('sonar-flags-raw');

    // Suivi de ligne
    DOM.lineStatus = document.getElementById('line-status');
    DOM.lineFlagsHex = document.getElementById('line-flags-hex');
    DOM.lineNodes = [];
    DOM.lineVals = [];
    DOM.lineRaws = [];
    DOM.lineCorrs = [];
    for (let i = 0; i < 5; ++i) {
        DOM.lineNodes.push(document.getElementById(`sensor-${i}`));
        DOM.lineVals.push(document.getElementById(`sval-${i}`));
        DOM.lineRaws.push(document.getElementById(`sraw-${i}`));
        DOM.lineCorrs.push(document.getElementById(`scorr-${i}`));
    }

    // Moteurs & Registres
    DOM.motorStateText = document.getElementById('motor-state-text');
    DOM.motorCompFlags = document.getElementById('motor-comp-flags');
    DOM.motorLeftApp = document.getElementById('motor-left-app');
    DOM.motorLeftTgt = document.getElementById('motor-left-tgt');
    DOM.motorLeftFill = document.getElementById('motor-left-fill');
    DOM.motorLeftEnc = document.getElementById('motor-left-enc');
    DOM.mLFlags = document.getElementById('m-l-flags');
    DOM.mLKp = document.getElementById('m-l-kp');
    DOM.mLKi = document.getElementById('m-l-ki');
    DOM.mLKd = document.getElementById('m-l-kd');

    DOM.motorRightApp = document.getElementById('motor-right-app');
    DOM.motorRightTgt = document.getElementById('motor-right-tgt');
    DOM.motorRightFill = document.getElementById('motor-right-fill');
    DOM.motorRightEnc = document.getElementById('motor-right-enc');
    DOM.mRFlags = document.getElementById('m-r-flags');
    DOM.mRKp = document.getElementById('m-r-kp');
    DOM.mRKi = document.getElementById('m-r-ki');
    DOM.mRKd = document.getElementById('m-r-kd');

    // Servomoteur Direction (CCR)
    DOM.steeringVal = document.getElementById('steering-val');
    DOM.steeringInd = document.getElementById('steering-indicator');
    DOM.stCurCcr = document.getElementById('st-cur-ccr');
    DOM.stTgtCcr = document.getElementById('st-tgt-ccr');
    DOM.stCoeff = document.getElementById('st-coeff');
    DOM.stFlags = document.getElementById('st-flags');

    // Matrice des drapeaux (Flags)
    DOM.flags = {
        // FSM
        fsmRdy: document.getElementById('flag-fsm-rdy'),
        fsmDisc: document.getElementById('flag-fsm-disc'),
        fsmReq: document.getElementById('flag-fsm-req'),
        fsmAuto: document.getElementById('flag-fsm-auto'),
        fsmHeadless: document.getElementById('flag-fsm-headless'),
        fsmStall: document.getElementById('flag-fsm-stall'),
        fsmAnom: document.getElementById('flag-fsm-anom'),
        fsmWarn: document.getElementById('flag-fsm-warn'),
        fsmCfg: document.getElementById('flag-fsm-cfg'),
        fsmDrv: document.getElementById('flag-fsm-drv'),
        fsmSock: document.getElementById('flag-fsm-sock'),
        fsmShm: document.getElementById('flag-fsm-shm'),
        // Services
        srvSys: document.getElementById('flag-srv-sys'),
        srvShm: document.getElementById('flag-srv-shm'),
        srvDev: document.getElementById('flag-srv-dev'),
        srvProc: document.getElementById('flag-srv-proc'),
        srvDaemons: document.getElementById('flag-srv-daemons'),
        srvSock: document.getElementById('flag-srv-sock'),
        srvDisc: document.getElementById('flag-srv-disc'),
        srvEth: document.getElementById('flag-srv-eth'),
        srvDhcp: document.getElementById('flag-srv-dhcp'),
        srvSsh: document.getElementById('flag-srv-ssh'),
        srvShell: document.getElementById('flag-srv-shell'),
        srvStall: document.getElementById('flag-srv-stall'),
        srvAnom: document.getElementById('flag-srv-anom'),
        // Périphériques
        motRdy: document.getElementById('flag-mot-rdy'),
        motDet: document.getElementById('flag-mot-det'),
        motEnc: document.getElementById('flag-mot-enc'),
        motPid: document.getElementById('flag-mot-pid'),
        srvPwm: document.getElementById('flag-srv-pwm'),
        srvRdy: document.getElementById('flag-srv-rdy'),
        snsObj: document.getElementById('flag-sns-obj'),
        snsAct: document.getElementById('flag-sns-act'),
        lineFol: document.getElementById('flag-line-fol'),
        lineLost: document.getElementById('flag-line-lost')
    };

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

function updateHex(el, key, num, width = 8) {
    const val = (num || 0) >>> 0;
    const hexStr = '0x' + val.toString(16).toUpperCase().padStart(width, '0');
    if (el && lastRendered[key] !== hexStr) {
        el.textContent = hexStr;
        if (val !== 0) {
            el.classList.remove('ok');
            el.classList.add('error');
        } else {
            el.classList.remove('error');
            el.classList.add('ok');
        }
        lastRendered[key] = hexStr;
    }
    return val !== 0;
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

    // 1. Numéro de séquence
    if (telem.seq !== undefined) {
        updateText(DOM.packetSeq, 'seq', telem.seq);
    }

    let hasAnyError = false;

    // 2. Diagnostics & Codes d'erreurs
    if (telem.fsm) {
        if (updateHex(DOM.errFsm, 'err_fsm', telem.fsm.error_code, 8)) hasAnyError = true;
    }
    if (telem.motors) {
        if (updateHex(DOM.errMotors, 'err_mot', telem.motors.error_code, 8)) hasAnyError = true;
        if (telem.motors.steering) {
            if (updateHex(DOM.errSteering, 'err_st', telem.motors.steering.error_code, 8)) hasAnyError = true;
        }
    }
    if (telem.aux_servo) {
        if (updateHex(DOM.errAuxServo, 'err_aux', telem.aux_servo.error_code, 8)) hasAnyError = true;
    }
    if (telem.sensors) {
        if (updateHex(DOM.errSensors, 'err_sens', telem.sensors.component_error_code, 4)) hasAnyError = true;
        if (telem.sensors.ultrasonic) {
            if (updateHex(DOM.errSonar, 'err_son', telem.sensors.ultrasonic.error_code, 8)) hasAnyError = true;
        }
    }
    if (telem.services) {
        if (updateHex(DOM.errHandler, 'err_hdl', telem.services.handler_error_code, 8)) hasAnyError = true;
    }

    // Bannière d'état global
    if (DOM.healthBanner && lastRendered['has_err'] !== hasAnyError) {
        lastRendered['has_err'] = hasAnyError;
        if (hasAnyError) {
            DOM.healthBanner.className = 'health-banner health-err';
            if (DOM.healthStatusText) DOM.healthStatusText.textContent = 'Anomalie Détectée — Consulter les Codes d\'Erreur';
        } else {
            DOM.healthBanner.className = 'health-banner health-ok';
            if (DOM.healthStatusText) DOM.healthStatusText.textContent = 'Système Nominal — Zéro Erreur Active';
        }
    }

    // 3. FSM
    if (telem.fsm) {
        const stateName = telem.fsm.state_name || 'Inconnu';
        const stateId = telem.fsm.state_id !== undefined ? telem.fsm.state_id : '--';
        updateText(DOM.fsmBadge, 'fsm_name', stateName);
        updateText(DOM.fsmId, 'fsm_id', stateId);

        const reqName = telem.fsm.requested_state_name || stateName;
        const reqId = telem.fsm.requested_state_id !== undefined ? telem.fsm.requested_state_id : stateId;
        updateText(DOM.fsmReqBadge, 'fsm_req_name', reqName);
        updateText(DOM.fsmReqId, 'fsm_req_id', reqId);

        const fsmFlags = valNum(telem.fsm.flags, 0);
        updateText(DOM.fsmFlagsHex, 'fsm_flags', '0x' + fsmFlags.toString(16).toUpperCase().padStart(4, '0'));

        // Drapeaux FSM individuels
        updateActive(DOM.flags.fsmDisc, 'f_fsm_disc', !!(fsmFlags & (1 << 0)));
        updateActive(DOM.flags.fsmAnom, 'f_fsm_anom', !!(fsmFlags & (1 << 1)));
        updateActive(DOM.flags.fsmWarn, 'f_fsm_warn', !!(fsmFlags & (1 << 2)));
        updateActive(DOM.flags.fsmCfg, 'f_fsm_cfg', !!(fsmFlags & (1 << 3)));
        updateActive(DOM.flags.fsmDrv, 'f_fsm_drv', !!(fsmFlags & (1 << 4)));
        updateActive(DOM.flags.fsmSock, 'f_fsm_sock', !!(fsmFlags & (1 << 5)));
        updateActive(DOM.flags.fsmShm, 'f_fsm_shm', !!(fsmFlags & (1 << 6)));
        updateActive(DOM.flags.fsmStall, 'f_fsm_stall', !!(fsmFlags & (1 << 7)));
        updateActive(DOM.flags.fsmReq, 'f_fsm_req', !!(fsmFlags & (1 << 8)));
        updateActive(DOM.flags.fsmRdy, 'f_fsm_rdy', !!(fsmFlags & (1 << 9)));
        updateActive(DOM.flags.fsmAuto, 'f_fsm_auto', !!(fsmFlags & (1 << 10)));
        updateActive(DOM.flags.fsmHeadless, 'f_fsm_head', !!(fsmFlags & (1 << 11)));
    }

    // 4. Sonar Distance & Flags
    if (telem.sensors && telem.sensors.ultrasonic) {
        const u = telem.sensors.ultrasonic;
        const dist = valNum(u.calc_dist_cm, 0);
        const raw = valNum(u.raw_dist_cm, 0);
        const uFlags = valNum(u.flags, 0);

        updateText(DOM.sonarDist, 's_dist', dist.toFixed(1));
        updateText(DOM.sonarRaw, 's_raw', raw.toFixed(1));
        updateText(DOM.sonarFlagsRaw, 's_flags', '0x' + uFlags.toString(16).toUpperCase().padStart(2, '0'));

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

        updateActive(DOM.flags.snsObj, 'f_sns_obj', !!(uFlags & 1));
        updateActive(DOM.flags.snsAct, 'f_sns_act', !!(uFlags & (1 << 1)));
    }

    // 5. Capteurs Suivi de Ligne
    if (telem.sensors && telem.sensors.line_follower) {
        const lf = telem.sensors.line_follower;
        const lFlags = valNum(lf.flags, 0);
        updateText(DOM.lineFlagsHex, 'line_flags', '0x' + lFlags.toString(16).toUpperCase().padStart(4, '0'));

        const sensors = Array.isArray(lf.sensors) ? lf.sensors : [];
        let anyDetected = false;
        for (let i = 0; i < sensors.length && i < 5; ++i) {
            const s = sensors[i] || {};
            const val = valNum(s.normalized, 0);
            const rawVal = valNum(s.raw, 0);
            const corr = valNum(s.corrector, 0);

            updateText(DOM.lineVals[i], `sval_${i}`, val.toFixed(2));
            updateText(DOM.lineRaws[i], `sraw_${i}`, `ADC: ${rawVal}`);
            updateText(DOM.lineCorrs[i], `scorr_${i}`, `Corr: ${corr.toFixed(1)}`);

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

        updateActive(DOM.flags.lineFol, 'f_line_fol', !!(lFlags & (1 << 3)));
        updateActive(DOM.flags.lineLost, 'f_line_lost', !!(lFlags & (1 << 4)));
    }

    // 6. Moteurs, Encodeurs & PWM Direction
    if (telem.motors) {
        const mState = telem.motors.state_name || 'Init';
        updateText(DOM.motorStateText, 'm_state', mState);
        const mCompFlags = valNum(telem.motors.component_flags, 0);
        updateText(DOM.motorCompFlags, 'm_comp_flags', '0x' + mCompFlags.toString(16).toUpperCase().padStart(4, '0'));

        // Moteur Gauche
        if (telem.motors.left) {
            const l = telem.motors.left;
            const appliedPct = Math.round(valNum(l.applied_throttle, 0) * 100);
            const targetPct = Math.round(valNum(l.target_throttle, 0) * 100);
            updateText(DOM.motorLeftApp, 'm_l_app', `${appliedPct}%`);
            updateText(DOM.motorLeftTgt, 'm_l_tgt', `${targetPct}%`);
            updateWidth(DOM.motorLeftFill, 'm_l_fill', `${Math.min(100, Math.max(0, appliedPct))}%`);
            updateText(DOM.motorLeftEnc, 'm_l_enc', l.encoder !== undefined ? l.encoder : 0);
            updateText(DOM.mLFlags, 'm_l_flags', '0x' + valNum(l.motor_flags, 0).toString(16).toUpperCase().padStart(2, '0'));

            if (l.pid) {
                updateText(DOM.mLKp, 'm_l_kp', valNum(l.pid.kp).toFixed(2));
                updateText(DOM.mLKi, 'm_l_ki', valNum(l.pid.ki).toFixed(2));
                updateText(DOM.mLKd, 'm_l_kd', valNum(l.pid.kd).toFixed(2));
            }

            const mFlags = valNum(l.motor_flags, 0);
            updateActive(DOM.flags.motDet, 'f_mot_det', !!(mFlags & 1));
            updateActive(DOM.flags.motEnc, 'f_mot_enc', !!(mFlags & (1 << 1)));
            updateActive(DOM.flags.motRdy, 'f_mot_rdy', !!(mFlags & (1 << 2)));
            updateActive(DOM.flags.motPid, 'f_mot_pid', !!(mFlags & (1 << 3)));
        }

        // Moteur Droit
        if (telem.motors.right) {
            const r = telem.motors.right;
            const appliedPct = Math.round(valNum(r.applied_throttle, 0) * 100);
            const targetPct = Math.round(valNum(r.target_throttle, 0) * 100);
            updateText(DOM.motorRightApp, 'm_r_app', `${appliedPct}%`);
            updateText(DOM.motorRightTgt, 'm_r_tgt', `${targetPct}%`);
            updateWidth(DOM.motorRightFill, 'm_r_fill', `${Math.min(100, Math.max(0, appliedPct))}%`);
            updateText(DOM.motorRightEnc, 'm_r_enc', r.encoder !== undefined ? r.encoder : 0);
            updateText(DOM.mRFlags, 'm_r_flags', '0x' + valNum(r.motor_flags, 0).toString(16).toUpperCase().padStart(2, '0'));

            if (r.pid) {
                updateText(DOM.mRKp, 'm_r_kp', valNum(r.pid.kp).toFixed(2));
                updateText(DOM.mRKi, 'm_r_ki', valNum(r.pid.ki).toFixed(2));
                updateText(DOM.mRKd, 'm_r_kd', valNum(r.pid.kd).toFixed(2));
            }
        }

        // Direction (Servo)
        if (telem.motors.steering) {
            const st = telem.motors.steering;
            const angle = valNum(st.current_angle, 0);
            updateText(DOM.steeringVal, 'st_val', `${angle.toFixed(1)}°`);
            const pointerPct = Math.min(100, Math.max(0, ((angle + 45.0) / 90.0) * 100));
            updateLeft(DOM.steeringInd, 'st_ind', `${pointerPct.toFixed(1)}%`);

            updateText(DOM.stCurCcr, 'st_cur_ccr', st.current_ccr !== undefined ? st.current_ccr : '--');
            updateText(DOM.stTgtCcr, 'st_tgt_ccr', st.target_ccr !== undefined ? st.target_ccr : '--');
            updateText(DOM.stCoeff, 'st_coeff', valNum(st.coefficient, 1.0).toFixed(2));
            const sFlags = valNum(st.flags, 0);
            updateText(DOM.stFlags, 'st_flags', '0x' + sFlags.toString(16).toUpperCase().padStart(4, '0'));

            updateActive(DOM.flags.srvPwm, 'f_srv_pwm', !!(sFlags & (1 << 7)));
            updateActive(DOM.flags.srvRdy, 'f_srv_rdy', !!(sFlags & (1 << 8)));
        }
    }

    // 7. Services & Superviseur des 4 Daemons
    if (telem.services) {
        updateText(DOM.servicesTimestamp, 'srv_ts', telem.services.timestamp || 0);
        updateText(DOM.servicesDaemonsCount, 'srv_d_count', telem.services.daemons_count || 0);

        const srvFlags = valNum(telem.services.flags, 0);
        updateActive(DOM.flags.srvDev, 'f_srv_dev', !!(srvFlags & (1 << 0)));
        updateActive(DOM.flags.srvShm, 'f_srv_shm', !!(srvFlags & (1 << 1)));
        updateActive(DOM.flags.srvProc, 'f_srv_proc', !!(srvFlags & (1 << 2)));
        updateActive(DOM.flags.srvSys, 'f_srv_sys', !!(srvFlags & (1 << 3)));
        updateActive(DOM.flags.srvDaemons, 'f_srv_dae', !!(srvFlags & (1 << 4)));
        updateActive(DOM.flags.srvEth, 'f_srv_eth', !!(srvFlags & (1 << 5)));
        updateActive(DOM.flags.srvAnom, 'f_srv_anom', !!(srvFlags & (1 << 7)));
        updateActive(DOM.flags.srvSock, 'f_srv_sock', !!(srvFlags & (1 << 8)));
        updateActive(DOM.flags.srvDisc, 'f_srv_disc', !!(srvFlags & (1 << 9)));
        updateActive(DOM.flags.srvStall, 'f_srv_stall', !!(srvFlags & (1 << 10)));
        updateActive(DOM.flags.srvShell, 'f_srv_shl', !!(srvFlags & (1 << 11)));
        updateActive(DOM.flags.srvSsh, 'f_srv_ssh', !!(srvFlags & (1 << 12)));
        updateActive(DOM.flags.srvDhcp, 'f_srv_dhcp', !!(srvFlags & (1 << 13)));

        if (Array.isArray(telem.services.daemons)) {
            for (let i = 0; i < telem.services.daemons.length && i < 4; ++i) {
                const d = telem.services.daemons[i];
                const dSlot = DOM.daemons[i];
                if (dSlot) {
                    const isOnline = d.status === 1;
                    const statusText = isOnline ? 'Actif' : 'Inactif';
                    updateText(dSlot.status, `d_${i}_st`, statusText);
                    updateText(dSlot.err, `d_${i}_err`, '0x' + valNum(d.error_code, 0).toString(16).toUpperCase().padStart(2, '0'));

                    if (dSlot.status && lastRendered[`d_${i}_st_col`] !== isOnline) {
                        dSlot.status.className = isOnline ? 'status-online' : 'status-offline';
                        lastRendered[`d_${i}_st_col`] = isOnline;
                    }
                    if (dSlot.container) {
                        if (isOnline) dSlot.container.classList.add('online');
                        else dSlot.container.classList.remove('online');
                    }
                }
            }
        }
    }

    // 8. Valeurs actuelles du robot dans le panneau de réglages
    updateParamsLive(telem);
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

async function sendServiceActive(service, active) {
    const action = active ? 'start' : 'stop';
    logConsole(`[TX] Commande ${action.toUpperCase()} transmise au service ${service} (CRC32)`, 'sent');
    try {
        const resp = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ service, action })
        });
        if (resp.ok) {
            const resJson = await resp.json();
            logConsole(`[RX] Ack: ${resJson.status}`, 'received');
        } else {
            logConsole(`[ERR] HTTP ${resp.status}`, 'error');
        }
    } catch (err) {
        logConsole(`[ERR] Échec: ${err.message}`, 'error');
    }
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
    while (DOM.cmdLogs.children.length > 200) {
        DOM.cmdLogs.removeChild(DOM.cmdLogs.firstChild);
    }
    DOM.cmdLogs.scrollTop = DOM.cmdLogs.scrollHeight;
}

// ==============================================================================
// RÉGLAGES DU ROBOT
// Un seul catalogue décrit chaque paramètre ; le panneau (titres, descriptions, champs, valeurs en direct) en est généré.
// Pour ajouter un réglage : une ligne dans PARAMS (le service doit accepter la clé, voir tests/test_params_catalogue.js).
//
// Champs d'un paramètre :
//   service  FSM | MOTORS | SENSORS     key      clé envoyée au service
//   group    groupe d'affichage (PARAM_GROUPS)
//   title    nom en français             desc     à quoi ça sert, ce qui se passe si on l'augmente ou le diminue
//   unit     unité affichée              def      valeur par défaut (celle de fsm.conf)
//   step     pas des flèches du champ    decimals décimales affichées
//   rec      [min, max] plage conseillée : seulement un avertissement, jamais un blocage
//   hard     [min, max] limites imposées par le robot (il ramène la valeur dedans) ; absent = aucune limite
//   live     chemin de la valeur actuelle dans la télémétrie ; absent = le robot ne la renvoie pas
//   options  liste déroulante [{value, label}] au lieu d'un champ numérique
//   dist     distance parcourue estimée (vitesse x durée) : {speed: m/s constant} ou {speedKey: clé d'une autre ligne}
// ==============================================================================
const PARAM_GROUPS = [
    { id: 'speed', title: 'Vitesse et accélération',
      intro: "Ce qui fixe la rapidité du robot sur la ligne. La bille sort de la coupelle quand l'accélération dépasse 0,735 m/s²." },
    { id: 'line', title: 'Suivi de ligne',
      intro: "Comment la direction réagit quand le robot s'éloigne de la ligne." },
    { id: 'obstacle', title: "Obstacle : ralentir et s'arrêter",
      intro: "Ce que fait le robot quand l'ultrason voit un obstacle, avant la manœuvre d'évitement : il ralentit, s'arrête, attend, puis décide." },
    { id: 'avoid', title: "Évitement d'obstacle (12 étapes)",
      intro: "Manœuvre sans capteurs pour contourner le cube : le robot recule, fait 3 virages et 3 lignes droites, puis avance jusqu'à retrouver la ligne. Distance parcourue = vitesse × durée." },
    { id: 'motors', title: 'Moteurs et direction', intro: "Réglages mécaniques de la propulsion et du servo de direction." },
    { id: 'sensors', title: 'Capteur de distance (ultrason)', intro: "Filtrage de la mesure de distance avant qu'elle n'arrive au FSM." },
    { id: 'loops', title: 'Fréquences des boucles (avancé)',
      intro: "Nombre de cycles par seconde de chaque service. Ne les changer que si on sait pourquoi : les gains du suivi de ligne sont réglés pour 50 Hz." },
];

const SIDE_OPTIONS = [{ value: -1, label: 'Gauche' }, { value: 1, label: 'Droite' }];
const FORWARD_SPEED_MS = 0.16;   // vitesse des virages et des lignes droites de l'évitement quand elle n'a pas son propre réglage

const PARAMS = [
    // ---- Vitesse et accélération ----
    { service: 'FSM', key: 'MAX_POWER_PCT', group: 'speed', title: 'Puissance moteur en ligne droite', unit: '%', def: 50, step: 1, decimals: 0,
      desc: "Puissance envoyée aux moteurs quand le robot est à la vitesse de croisière : c'est elle qui fixe la vitesse réelle. 50 % correspond à environ 0,32 m/s (valeur validée). Plus haut = plus vite, mais la bille risque de tomber. N'affecte pas l'évitement.",
      rec: [20, 75], hard: [5, 100], live: 'fsm.config.max_power_pct' },
    { service: 'FSM', key: 'STRAIGHT_CRUISE_SPEED', group: 'speed', title: 'Vitesse de référence (croisière)', unit: 'm/s', def: 0.32, step: 0.01, decimals: 2,
      desc: "Vitesse de référence du FSM. La puissance des moteurs vaut vitesse ÷ croisière × la puissance ci-dessus : en ligne droite le robot est donc toujours à la puissance réglée. Une croisière plus grande rend surtout les virages plus lents par rapport à la ligne droite.",
      rec: [0.15, 0.45], hard: [0.05, 2], live: 'fsm.config.straight_cruise_speed' },
    { service: 'FSM', key: 'MIN_CURVE_SPEED', group: 'speed', title: 'Vitesse minimale en virage serré', unit: 'm/s', def: 0.20, step: 0.01, decimals: 2,
      desc: "Plancher de vitesse dans les virages. Sert aussi quand la ligne est perdue : le robot garde son dernier braquage à cette vitesse en cherchant le ruban.",
      rec: [0.1, 0.3], live: 'fsm.config.min_curve_speed' },
    { service: 'FSM', key: 'CURVE_SAFETY_FACTOR', group: 'speed', title: 'Marge de sécurité en virage', unit: '', def: 0.8, step: 0.05, decimals: 2,
      desc: "Vitesse en virage = marge × √(accélération max × rayon du virage). Sous 1, le robot ralentit plus dans les virages et la bille est protégée ; au-dessus de 1, il dépasse la limite théorique et la bille peut tomber.",
      rec: [0.5, 1], hard: [0.05, 5], live: 'fsm.config.curve_safety_factor' },
    { service: 'FSM', key: 'MAX_ACCELERATION', group: 'speed', title: 'Accélération maximale', unit: 'm/s²', def: 0.735, step: 0.05, decimals: 3,
      desc: "Limite de la rampe de vitesse : départ, freinage, ralentissement devant un obstacle. 0,735 m/s² est le seuil critique de la bille ; au-dessus, elle sort de la coupelle.",
      rec: [0.3, 0.735], live: 'fsm.config.max_acceleration' },

    // ---- Suivi de ligne ----
    { service: 'FSM', key: 'LINE_KP', group: 'line', title: 'Réactivité de la direction (gain P)', unit: '', def: 0.58, step: 0.01, decimals: 2,
      desc: "Plus il est grand, plus le robot braque fort quand il s'éloigne de la ligne. Trop grand : il zigzague. Trop petit : il sort des virages.",
      rec: [0.3, 1], live: 'fsm.config.line_kp' },
    { service: 'FSM', key: 'LINE_KD', group: 'line', title: 'Amortissement de la direction (gain D)', unit: '', def: 0.035, step: 0.005, decimals: 3,
      desc: "Freine les corrections quand l'erreur change vite : calme le zigzag. Trop grand : la direction devient nerveuse au moindre bruit des capteurs.",
      rec: [0, 0.1], live: 'fsm.config.line_kd' },

    // ---- Obstacle : ralentir et s'arrêter ----
    { service: 'FSM', key: 'OBSTACLE_SLOWDOWN_DIST_CM', group: 'obstacle', title: 'Distance de ralentissement', unit: 'cm', def: 45, step: 5, decimals: 0,
      desc: "Le robot commence à ralentir quand l'obstacle est plus près que cette distance. Il règle sa vitesse pour pouvoir s'arrêter à la distance d'arrêt sans dépasser l'accélération maximale.",
      rec: [30, 100], hard: [5, 400] },
    { service: 'FSM', key: 'OBSTACLE_STOP_DIST_CM', group: 'obstacle', title: "Distance d'arrêt", unit: 'cm', def: 20, step: 1, decimals: 0,
      desc: "Le robot s'arrête quand l'obstacle est à cette distance. Trop petit : il risque de toucher le cube avant de s'arrêter.",
      rec: [10, 40], hard: [2, 400], live: 'fsm.config.obstacle_stop_dist_cm' },
    { service: 'FSM', key: 'OBSTACLE_AVOID_MARGIN_CM', group: 'obstacle', title: "Marge avant l'évitement", unit: 'cm', def: 5, step: 1, decimals: 0,
      desc: "Après l'arrêt, le robot évite l'obstacle s'il est encore à (distance d'arrêt + cette marge). Sinon, il considère la voie libre et reprend la ligne.",
      rec: [0, 20], hard: [0, 100] },
    { service: 'FSM', key: 'OBSTACLE_STOP_WAIT_S', group: 'obstacle', title: "Attente après l'arrêt", unit: 's', def: 0.2, step: 0.1, decimals: 1,
      desc: "Temps d'attente à l'arrêt avant de décider : éviter ou repartir. Plus long = laisse le temps à un obstacle mobile de s'écarter.",
      rec: [0, 2], hard: [0, 10] },
    { service: 'FSM', key: 'AVOID_SIDE', group: 'obstacle', title: 'Côté du contournement', unit: '', def: 1, options: SIDE_OPTIONS, decimals: 0,
      desc: "Du côté où le robot passe pour contourner l'obstacle : le premier virage de l'évitement va de ce côté.",
      live: 'fsm.config.avoid_side' },

    // ---- Évitement d'obstacle (12 étapes) ----
    { service: 'FSM', key: 'AVOID_REVERSE_DIST_M', group: 'avoid', title: 'Étape 1 · Reculer', unit: 'm', def: 0.25, step: 0.01, decimals: 2,
      desc: "Distance de recul avant de contourner, pour se donner de la place.",
      rec: [0.1, 0.5], hard: [0.02, 3], live: 'fsm.config.avoidance.reverse_dist_m' },
    { service: 'FSM', key: 'AVOID_ANGLE_1_DEG', group: 'avoid', title: 'Étape 2 · Angle du 1er virage', unit: '°', def: 45, step: 1, decimals: 0,
      desc: "Braquage des roues vers le côté choisi. 45° est le braquage maximal du robot.",
      rec: [15, 45], hard: [0, 45], live: 'fsm.config.avoidance.angle_1_deg' },
    { service: 'FSM', key: 'AVOID_TIME_1_S', group: 'avoid', title: 'Étape 3 · Durée du 1er virage', unit: 's', def: 1.1, step: 0.05, decimals: 2,
      desc: "Temps pendant lequel le robot avance roues braquées. Plus long = il tourne davantage (le cap change d'environ l'angle) et s'écarte plus du cube.",
      rec: [0.3, 3], hard: [0.05, 30], live: 'fsm.config.avoidance.time_1_s', dist: { speed: FORWARD_SPEED_MS } },
    { service: 'FSM', key: 'AVOID_DIAG_TIME_S', group: 'avoid', title: 'Étapes 4-5 · Durée de la diagonale', unit: 's', def: 1.1, step: 0.05, decimals: 2,
      desc: "Roues droites : le robot avance en ligne droite en s'écartant du cube. Plus long = il s'en écarte davantage.",
      rec: [0.3, 3], hard: [0.05, 30], live: 'fsm.config.avoidance.diag_time_s', dist: { speedKey: 'AVOID_DIAG_SPEED' } },
    { service: 'FSM', key: 'AVOID_DIAG_SPEED', group: 'avoid', title: 'Étapes 4-5 · Vitesse de la diagonale', unit: 'm/s', def: 0.16, step: 0.01, decimals: 2,
      desc: "Vitesse pendant la diagonale. Les durées des autres étapes ont été calibrées à 0,16 m/s : si tu changes cette vitesse, il faudra peut-être ajuster la durée.",
      rec: [0.12, 0.25], hard: [0.05, 0.6], live: 'fsm.config.avoidance.diag_speed' },
    { service: 'FSM', key: 'AVOID_ANGLE_2_DEG', group: 'avoid', title: 'Étape 6 · Angle du 2e virage', unit: '°', def: 45, step: 1, decimals: 0,
      desc: "Braquage vers l'autre côté pour se remettre parallèle à la ligne.",
      rec: [15, 45], hard: [0, 45], live: 'fsm.config.avoidance.angle_2_deg' },
    { service: 'FSM', key: 'AVOID_TIME_2_S', group: 'avoid', title: 'Étape 7 · Durée du 2e virage', unit: 's', def: 0.7, step: 0.05, decimals: 2,
      desc: "Temps de ce virage. Il doit redresser le robot : trop court, il reste en biais ; trop long, il repart vers le cube.",
      rec: [0.2, 2], hard: [0.05, 30], live: 'fsm.config.avoidance.time_2_s', dist: { speed: FORWARD_SPEED_MS } },
    { service: 'FSM', key: 'AVOID_PARALLEL_TIME_S', group: 'avoid', title: 'Étape 8 · Durée en ligne droite parallèle', unit: 's', def: 1, step: 0.05, decimals: 2,
      desc: "Le robot est parallèle à la ligne et dépasse le cube. Plus long = il va plus loin avant de revenir vers la ligne.",
      rec: [0.2, 4], hard: [0.05, 30], live: 'fsm.config.avoidance.parallel_time_s', dist: { speed: FORWARD_SPEED_MS } },
    { service: 'FSM', key: 'AVOID_ANGLE_3_DEG', group: 'avoid', title: 'Étape 9 · Angle du 3e virage', unit: '°', def: 45, step: 1, decimals: 0,
      desc: "Braquage vers la ligne pour y revenir.",
      rec: [15, 45], hard: [0, 45], live: 'fsm.config.avoidance.angle_3_deg' },
    { service: 'FSM', key: 'AVOID_TIME_3_S', group: 'avoid', title: 'Étape 10 · Durée du 3e virage', unit: 's', def: 0.7, step: 0.05, decimals: 2,
      desc: "Temps de ce virage vers la ligne. Plus long = le robot arrive à la ligne plus en biais.",
      rec: [0.2, 2], hard: [0.05, 30], live: 'fsm.config.avoidance.time_3_s', dist: { speed: FORWARD_SPEED_MS } },
    { service: 'FSM', key: 'AVOID_RETURN_TIME_S', group: 'avoid', title: 'Étapes 11-12 · Durée maximale de recherche de la ligne', unit: 's', def: 3.5, step: 0.1, decimals: 2,
      desc: "Roues droites, le robot avance jusqu'à voir la ligne. S'il ne la trouve pas avant cette durée, il s'arrête en erreur (état Failure).",
      rec: [1, 5], hard: [0.5, 60], live: 'fsm.config.avoidance.return_time_s', dist: { speedKey: 'AVOID_RETURN_SPEED', prefix: 'au plus' } },
    { service: 'FSM', key: 'AVOID_RETURN_SPEED', group: 'avoid', title: 'Étapes 11-12 · Vitesse de recherche de la ligne', unit: 'm/s', def: 0.16, step: 0.01, decimals: 2,
      desc: "Vitesse pendant la recherche de la ligne. Trop rapide : il peut dépasser la ligne sans la voir.",
      rec: [0.12, 0.25], hard: [0.05, 0.6], live: 'fsm.config.avoidance.return_speed' },

    // ---- Moteurs et direction ----
    { service: 'MOTORS', key: 'STEERING_OFFSET', group: 'motors', title: 'Trim de la direction', unit: '°', def: 0, step: 0.5, decimals: 1,
      desc: "Corrige un défaut d'alignement des roues : si le robot dévie d'un côté en ligne droite, décale la direction de quelques degrés dans l'autre sens.",
      rec: [-10, 10], live: 'motors.config.steering_offset' },
    { service: 'MOTORS', key: 'INVERT', group: 'motors', title: 'Sens des moteurs', unit: '', def: 0, decimals: 0,
      options: [{ value: 0, label: 'Normal' }, { value: 1, label: 'Inversé' }],
      desc: "À inverser si le robot recule alors qu'il devrait avancer (fils des moteurs branchés à l'envers).",
      live: 'motors.config.invert' },

    // ---- Capteur de distance ----
    { service: 'SENSORS', key: 'FILTER_ALPHA', group: 'sensors', title: 'Lissage de la distance', unit: '', def: 0.25, step: 0.05, decimals: 2,
      desc: "Entre 0,01 et 1. Petit = mesure très lissée mais lente à réagir à un obstacle ; 1 = aucun lissage (mesure brute, plus bruitée). Cette limite est mathématique.",
      rec: [0.05, 1], hard: [0.01, 1], live: 'sensors.config.filter_alpha' },
    { service: 'SENSORS', key: 'MEDIAN_WINDOW', group: 'sensors', title: 'Fenêtre du filtre médian', unit: 'mesures', def: 5, step: 1, decimals: 0,
      desc: "Nombre de mesures comparées pour ignorer une valeur aberrante isolée. Plus grand = plus robuste, mais la détection d'un obstacle est plus lente. Limite imposée par le robot : 15.",
      rec: [3, 9], hard: [1, 15], live: 'sensors.config.median_window' },

    // ---- Fréquences des boucles ----
    { service: 'FSM', key: 'HZ', group: 'loops', title: 'Fréquence de la boucle FSM', unit: 'Hz', def: 50, step: 5, decimals: 0,
      desc: "Nombre de décisions du FSM par seconde. LINE_KP et LINE_KD sont réglés pour 50 Hz : à 100 Hz, l'évitement échouait en simulation.",
      rec: [20, 100], hard: [10, 10000], live: 'fsm.config.hz' },
    { service: 'MOTORS', key: 'HZ', group: 'loops', title: 'Fréquence de la boucle Motors', unit: 'Hz', def: 100, step: 10, decimals: 0,
      desc: "Nombre de mises à jour des moteurs et du servo par seconde.",
      rec: [50, 200], hard: [10, 10000], live: 'motors.config.hz' },
    { service: 'SENSORS', key: 'HZ', group: 'loops', title: 'Fréquence de la boucle Sensors', unit: 'Hz', def: 50, step: 5, decimals: 0,
      desc: "Nombre de lectures des capteurs par seconde.",
      rec: [20, 100], hard: [10, 10000], live: 'sensors.config.hz' },
];

const APPLY_CONFIRM_TIMEOUT_MS = 4000;   // au-delà, si le robot n'a pas la valeur demandée, on affiche ce qu'il a gardé
const paramState = {};                    // par paramètre : champ, étiquette, dernière demande envoyée, valeur modifiée à la main

function paramId(p) { return `${p.service}.${p.key}`; }

function getPath(obj, path) {
    return path.split('.').reduce((o, k) => (o === undefined || o === null ? undefined : o[k]), obj);
}

function el(tag, className, text) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    return node;
}

// Nombre sans zéros inutiles (100.0 -> 100) suivi de son unité (le degré est collé au nombre)
function withUnit(v, unit, decimals = 2) {
    const n = String(Number(Number(v).toFixed(decimals)));
    if (!unit) return n;
    return unit === '°' ? `${n}°` : `${n} ${unit}`;
}

function fmtRange(range, unit, decimals = 2) {
    const lo = withUnit(range[0], '', decimals);
    return `${lo} à ${withUnit(range[1], unit, decimals)}`;
}

function sameValue(a, b) {
    return Math.abs(a - b) <= Math.max(1e-4, 1e-3 * Math.abs(b));
}

// ------------------------------------------------------------------------------
// Construction du panneau
// ------------------------------------------------------------------------------
function buildParamPanel() {
    const host = document.getElementById('param-groups');
    if (!host) return;

    for (const group of PARAM_GROUPS) {
        const details = el('details', 'param-group');
        details.dataset.group = group.id;
        details.open = group.id !== 'loops';
        const summary = el('summary', 'param-group-title');
        summary.appendChild(el('span', 'param-group-name', group.title));
        const count = PARAMS.filter((p) => p.group === group.id).length;
        summary.appendChild(el('span', 'param-group-count', `${count} réglage${count > 1 ? 's' : ''}`));
        const applyAll = el('button', 'btn-sm param-apply-all', 'Appliquer les valeurs modifiées');
        applyAll.type = 'button';
        applyAll.addEventListener('click', (e) => { e.preventDefault(); applyGroup(group.id); });
        summary.appendChild(applyAll);
        details.appendChild(summary);
        details.appendChild(el('p', 'param-group-intro', group.intro));

        const list = el('div', 'param-list');
        for (const p of PARAMS.filter((x) => x.group === group.id)) list.appendChild(buildParamRow(p));
        details.appendChild(list);
        host.appendChild(details);
    }

    const search = document.getElementById('param-search');
    if (search) search.addEventListener('input', () => filterParams(search.value));
    PARAMS.forEach(updateHint);
}

function buildParamRow(p) {
    const id = paramId(p);
    const row = el('div', 'param-row');
    row.dataset.param = id;
    row.dataset.search = `${p.title} ${p.key} ${p.desc} ${p.service}`.toLowerCase();

    // Colonne de gauche : nom, description, plages, valeur actuelle
    const info = el('div', 'param-info');
    const head = el('div', 'param-head');
    head.appendChild(el('span', 'param-title', p.title));
    head.appendChild(el('code', 'param-key', p.key));
    head.appendChild(el('span', `param-service service-${p.service.toLowerCase()}`, p.service));
    info.appendChild(head);
    info.appendChild(el('div', 'param-desc', p.desc));

    const defaultText = p.options ? labelOf(p, p.def) : withUnit(p.def, p.unit, p.decimals + 1);
    const meta = [`Défaut : ${defaultText}`];
    if (p.rec) meta.push(`Conseillé : ${fmtRange(p.rec, p.unit, p.decimals + 1)}`);
    if (p.hard) meta.push(`Limite du robot : ${fmtRange(p.hard, p.unit, p.decimals + 1)}`);
    else if (!p.options) meta.push('Aucune limite imposée par le robot');
    info.appendChild(el('div', 'param-meta', meta.join(' · ')));

    const hint = el('div', 'param-hint');
    info.appendChild(hint);
    const live = el('span', p.live ? 'live-tag' : 'live-tag nolive', p.live ? 'Actuel (robot) : --' : 'Pas de retour du robot pour ce réglage');
    info.appendChild(live);
    row.appendChild(info);

    // Colonne de droite : champ, unité, boutons
    const controls = el('div', 'param-controls');
    let input;
    if (p.options) {
        input = el('select', 'param-input');
        for (const o of p.options) {
            const opt = el('option', '', o.label);
            opt.value = String(o.value);
            input.appendChild(opt);
        }
        input.value = String(p.def);
    } else {
        input = el('input', 'param-input');
        input.type = 'number';
        input.step = String(p.step || 'any');   // pas de min ni de max : aucune limite côté navigateur
        input.value = String(p.def);
    }
    input.id = `cfg-${p.service.toLowerCase()}-${p.key.toLowerCase()}`;
    input.setAttribute('aria-label', `${p.title} (${p.key})`);
    controls.appendChild(input);
    if (p.unit && !p.options) controls.appendChild(el('span', 'param-unit', p.unit));

    const apply = el('button', 'btn-sm', 'Appliquer');
    apply.type = 'button';
    apply.addEventListener('click', () => applyParam(p));
    const reset = el('button', 'btn-sm btn-icon', '↺');
    reset.type = 'button';
    reset.title = 'Remettre la valeur par défaut dans le champ (sans l’envoyer)';
    reset.addEventListener('click', () => { input.value = String(p.def); markTouched(p); updateHint(p); updateDependentHints(p); });
    controls.appendChild(apply);
    controls.appendChild(reset);
    row.appendChild(controls);

    const onEdit = () => { markTouched(p); updateHint(p); updateDependentHints(p); };
    input.addEventListener('input', onEdit);
    input.addEventListener('change', onEdit);
    input.addEventListener('keydown', (e) => { if (e.key === 'Enter') applyParam(p); });

    paramState[id] = { p, row, input, hint, live, touched: false, sent: null, confirmedUntil: 0, liveSeen: false };
    return row;
}

function labelOf(p, value) {
    const o = (p.options || []).find((x) => sameValue(x.value, value));
    return o ? o.label : String(value);
}

function markTouched(p) {
    const st = paramState[paramId(p)];
    st.touched = true;
    st.row.classList.add('dirty');
}

// ------------------------------------------------------------------------------
// Avertissements sous chaque champ (plage conseillée, limites du robot, distance estimée)
// ------------------------------------------------------------------------------
function currentInputValue(p) {
    const v = parseFloat(paramState[paramId(p)].input.value);
    return isNaN(v) ? null : v;
}

function currentInputValueByKey(key) {
    const other = PARAMS.find((x) => x.key === key && x.service === 'FSM');
    return other ? currentInputValue(other) : null;
}

// Les durées affichent une distance qui dépend d'une vitesse réglée ailleurs : on les recalcule quand cette vitesse change
function updateDependentHints(p) {
    PARAMS.filter((q) => q.dist && q.dist.speedKey === p.key && q.service === p.service).forEach(updateHint);
}

function updateHint(p) {
    const st = paramState[paramId(p)];
    const v = currentInputValue(p);
    const parts = [];
    let level = '';

    if (v === null) {
        parts.push('Saisis un nombre.');
        level = 'warn';
    } else if (p.hard && (v < p.hard[0] || v > p.hard[1])) {
        const kept = Math.min(p.hard[1], Math.max(p.hard[0], v));
        parts.push(`⛔ Le robot ramènera cette valeur à ${withUnit(kept, p.unit, p.decimals + 1)} (limite ${fmtRange(p.hard, p.unit, p.decimals + 1)}).`);
        level = 'limit';
    } else if (p.rec && (v < p.rec[0] || v > p.rec[1])) {
        parts.push(`⚠ Hors de la plage conseillée (${fmtRange(p.rec, p.unit, p.decimals + 1)}) : le robot l'accepte quand même.`);
        level = 'warn';
    }

    if (v !== null && p.dist) {
        const speed = p.dist.speed !== undefined ? p.dist.speed : currentInputValueByKey(p.dist.speedKey);
        if (speed !== null && speed !== undefined) {
            parts.push(`≈ ${p.dist.prefix ? p.dist.prefix + ' ' : ''}${(v * speed * 100).toFixed(0)} cm parcourus (${withUnit(speed, 'm/s')} × ${withUnit(v, 's')}).`);
        }
    }

    st.hint.textContent = parts.join(' ');
    st.hint.className = 'param-hint' + (level ? ' ' + level : '');
    st.input.classList.toggle('out-of-range', level === 'warn' || level === 'limit');
}

// ------------------------------------------------------------------------------
// Recherche
// ------------------------------------------------------------------------------
function filterParams(text) {
    const q = text.trim().toLowerCase();
    document.querySelectorAll('.param-group').forEach((group) => {
        let visible = 0;
        group.querySelectorAll('.param-row').forEach((row) => {
            const match = !q || row.dataset.search.includes(q);
            row.hidden = !match;
            if (match) visible++;
        });
        group.hidden = visible === 0;
        if (q && visible > 0) group.open = true;
    });
}

// ------------------------------------------------------------------------------
// Valeurs actuelles du robot (appelé à chaque trame de télémétrie)
// ------------------------------------------------------------------------------
function updateParamsLive(telem) {
    for (const id in paramState) {
        const st = paramState[id];
        const p = st.p;
        if (!p.live) continue;
        const robot = valNum(getPath(telem, p.live), NaN);
        if (isNaN(robot)) continue;

        // Premier retour du robot : le champ part de la vraie valeur, pas de la valeur par défaut
        if (!st.liveSeen) {
            st.liveSeen = true;
            if (!st.touched) {
                st.input.value = p.options ? String(robot) : String(Number(robot.toFixed(Math.max(p.decimals + 1, 3))));
                updateHint(p);
                updateDependentHints(p);
            }
        }

        const text = formatLive(p, robot, st);
        updateText(st.live, `live_${id}`, text.message);
        if (lastRendered[`livecls_${id}`] !== text.cls) {
            st.live.className = 'live-tag' + (text.cls ? ' ' + text.cls : '');
            lastRendered[`livecls_${id}`] = text.cls;
        }

        // « modifié » = le champ est différent de ce que le robot a vraiment
        const inputVal = currentInputValue(p);
        st.row.classList.toggle('dirty', st.touched && inputVal !== null && !sameValue(inputVal, robot));
    }
}

function formatLive(p, robot, st) {
    const shown = p.options ? labelOf(p, robot) : withUnit(robot, p.unit, p.decimals);
    const req = st.sent;
    if (!req && st.confirmedUntil > Date.now()) {
        return { message: `✓ Confirmé par le robot : ${shown}`, cls: 'synced' };
    }
    if (req) {
        if (sameValue(robot, req.value)) {
            st.sent = null;                                          // fini : on n'attend plus rien de ce réglage
            st.confirmedUntil = Date.now() + APPLY_CONFIRM_TIMEOUT_MS;
            return { message: `✓ Confirmé par le robot : ${shown}`, cls: 'synced' };
        }
        if (Date.now() - req.time < APPLY_CONFIRM_TIMEOUT_MS) {
            return { message: `⏳ Envoi… (actuel : ${shown})`, cls: 'pending' };
        }
        const asked = p.options ? labelOf(p, req.value) : withUnit(req.value, p.unit, p.decimals);
        return { message: `⚠ Le robot a gardé ${shown} au lieu de ${asked} (limite ou refus)`, cls: 'mismatch' };
    }
    const inputVal = currentInputValue(p);
    if (inputVal !== null && sameValue(robot, inputVal)) {
        return { message: `✓ Actuel (robot) : ${shown}`, cls: 'synced' };
    }
    return { message: `Actuel (robot) : ${shown}`, cls: '' };
}

// ------------------------------------------------------------------------------
// Envoi des réglages
// ------------------------------------------------------------------------------
async function sendConfig(service, key, value) {
    logConsole(`[TX] Réglage : ${service}.${key} = ${value}`, 'sent');
    try {
        const resp = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ service, key, value })
        });
        if (resp.ok) {
            const resJson = await resp.json();
            logConsole(`[RX] Ack serveur : ${resJson.command || 'OK'}`, 'received');
            return true;
        }
        logConsole(`[ERR] HTTP ${resp.status} lors de l'envoi de ${key}`, 'error');
    } catch (err) {
        logConsole(`[ERR] Erreur de communication : ${err.message}`, 'error');
    }
    return false;
}

async function applyParam(p) {
    const st = paramState[paramId(p)];
    const value = currentInputValue(p);
    if (value === null) {
        logConsole(`[ERR] Valeur invalide saisie pour ${p.service}.${p.key}`, 'error');
        return;
    }
    // Valeur que le robot devrait avoir après avoir appliqué ses propres limites
    const expected = p.hard ? Math.min(p.hard[1], Math.max(p.hard[0], value)) : value;
    st.sent = { value: expected, time: Date.now() };
    st.touched = false;
    st.row.classList.remove('dirty');
    updateText(st.live, `live_${paramId(p)}`, '⏳ Envoi…');
    st.live.className = 'live-tag pending';
    lastRendered[`livecls_${paramId(p)}`] = 'pending';
    const ok = await sendConfig(p.service, p.key, value);
    if (!p.live) {
        // Le robot ne renvoie pas cette valeur : on ne peut pas la confirmer
        const unit = p.unit ? ' ' + p.unit : '';
        st.live.textContent = ok ? `Envoyé : ${value}${unit} (le robot ne renvoie pas cette valeur : voir la console du FSM)` : '⚠ Envoi échoué';
        st.live.className = 'live-tag' + (ok ? ' synced' : ' mismatch');
        lastRendered[`live_${paramId(p)}`] = st.live.textContent;
        lastRendered[`livecls_${paramId(p)}`] = ok ? 'synced' : 'mismatch';
        st.sent = null;
    }
}

// Applique d'un coup toutes les lignes d'un groupe dont la valeur a été modifiée à la main
async function applyGroup(groupId) {
    const changed = PARAMS.filter((p) => p.group === groupId && paramState[paramId(p)].touched);
    if (changed.length === 0) {
        logConsole('[INFO] Aucune valeur modifiée dans ce groupe.', 'system');
        return;
    }
    for (const p of changed) {
        await applyParam(p);
    }
}

// Réglage libre : n'importe quelle clé sur n'importe quel service (pour les clés absentes du catalogue)
function applyCustomConfig() {
    const srv = document.getElementById('cfg-custom-srv');
    const keyInput = document.getElementById('cfg-custom-key');
    const valInput = document.getElementById('cfg-custom-val');
    if (!srv || !keyInput || !valInput) return;
    const key = keyInput.value.trim();
    const val = parseFloat(valInput.value);
    if (!key || isNaN(val)) {
        logConsole('[ERR] Veuillez renseigner une clé valide et une valeur numérique', 'error');
        return;
    }
    sendConfig(srv.value, key, val);
}
