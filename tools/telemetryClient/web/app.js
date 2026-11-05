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

    // Éléments de retour télémétrique en direct pour les configurations
    DOM.live = {
        fsmCruise: document.getElementById('live-fsm-cruise'),
        fsmMinCurve: document.getElementById('live-fsm-mincurve'),
        fsmSafety: document.getElementById('live-fsm-safety'),
        fsmAccel: document.getElementById('live-fsm-accel'),
        fsmKp: document.getElementById('live-fsm-kp'),
        fsmKd: document.getElementById('live-fsm-kd'),
        fsmDist: document.getElementById('live-fsm-dist'),
        fsmReverse: document.getElementById('live-fsm-reverse'),
        fsmSide: document.getElementById('live-fsm-side'),
        fsmAutostart: document.getElementById('live-fsm-autostart'),
        fsmHz: document.getElementById('live-fsm-hz'),
        motOffset: document.getElementById('live-mot-offset'),
        motInvert: document.getElementById('live-mot-invert'),
        motHz: document.getElementById('live-mot-hz'),
        snsAlpha: document.getElementById('live-sns-alpha'),
        snsWindow: document.getElementById('live-sns-window'),
        snsHz: document.getElementById('live-sns-hz')
    };
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

    // 8. Mise à jour des valeurs actuelles réelles dans les contrôles de configuration
    if (DOM.live) {
        if (telem.motors && telem.motors.left) {
            const throttlePct = Math.round(valNum(telem.motors.left.applied_throttle, 0) * 100);
            const approxSpeedMs = ((throttlePct / 50.0) * 0.32).toFixed(2);
            updateText(DOM.live.fsmCruise, 'live_fsm_cruise', `Actuel: ~${approxSpeedMs} m/s (${throttlePct}%)`);
            updateText(DOM.live.fsmMinCurve, 'live_fsm_mincurve', `Consigne: ${Math.round(valNum(telem.motors.left.target_throttle, 0) * 100)}%`);
        }
        if (telem.fsm) {
            updateText(DOM.live.fsmSafety, 'live_fsm_safety', `État FSM: ${telem.fsm.state_name || 'Inconnu'}`);
            updateText(DOM.live.fsmSide, 'live_fsm_side', (telem.fsm.flags & (1 << 10)) ? `Évitement Auto Activé` : `Actuel: Nominal`);
        }
        if (telem.sensors && telem.sensors.ultrasonic) {
            const curDist = valNum(telem.sensors.ultrasonic.calc_dist_cm, 0).toFixed(1);
            const rawDist = valNum(telem.sensors.ultrasonic.raw_dist_cm, 0).toFixed(1);
            updateText(DOM.live.fsmDist, 'live_fsm_dist', `Sonar actuel: ${curDist} cm (brut: ${rawDist} cm)`);
            updateText(DOM.live.snsAlpha, 'live_sns_alpha', `Calculé: ${curDist} cm`);
        }
        if (telem.sensors && telem.sensors.line_follower) {
            const lf = telem.sensors.line_follower;
            const anyDetected = (lf.flags & (1 << 3)) !== 0;
            updateText(DOM.live.fsmKp, 'live_fsm_kp', anyDetected ? `Suivi actif (Ligne OK)` : `Recherche / Ligne perdue`);
            if (Array.isArray(lf.sensors) && lf.sensors.length > 2) {
                const centerCorr = valNum(lf.sensors[2].corrector, 0).toFixed(2);
                updateText(DOM.live.fsmKd, 'live_fsm_kd', `Corr. Centre: ${centerCorr}`);
            }
        }
        if (telem.motors && telem.motors.steering) {
            const st = telem.motors.steering;
            const curAngle = valNum(st.current_angle, 0).toFixed(1);
            const ccr = st.current_ccr !== undefined ? st.current_ccr : '--';
            const coeff = valNum(st.coefficient, 1.0).toFixed(2);
            updateText(DOM.live.motOffset, 'live_mot_offset', `Angle: ${curAngle}° | CCR: ${ccr} | Trim: ${coeff}`);
        }
        if (telem.motors) {
            updateText(DOM.live.motInvert, 'live_mot_invert', `État Moteur: ${telem.motors.state_name || 'Inconnu'}`);
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

// Evitement d'obstacle : distance parcourue en diagonale = vitesse x duree (memes bornes que le FSM)
function avoidDistanceText(timeId, speedId, tagId, label, tMin, tMax, vMin, vMax) {
    const timeInput = document.getElementById(timeId);
    const speedInput = document.getElementById(speedId);
    const tag = document.getElementById(tagId);
    if (!timeInput || !speedInput || !tag) return;
    const t = parseFloat(timeInput.value);
    const v = parseFloat(speedInput.value);
    if (isNaN(t) || isNaN(v)) {
        tag.textContent = `${label} : -- cm`;
        return;
    }
    const tc = Math.min(tMax, Math.max(tMin, t));
    const vc = Math.min(vMax, Math.max(vMin, v));
    const clamped = (tc !== t || vc !== v) ? ' (valeurs ramenées aux bornes du FSM)' : '';
    tag.textContent = `${label} : ~${(tc * vc * 100).toFixed(0)} cm${clamped}`;
}

function updateAvoidDistance() {
    avoidDistanceText('cfg-avoid-diag-time', 'cfg-avoid-diag-speed', 'live-avoid-dist',
                      'Distance diagonale', 0.3, 3.0, 0.12, 0.25);
    avoidDistanceText('cfg-avoid-ret-time', 'cfg-avoid-ret-speed', 'live-avoid-return-dist',
                      'Distance maximale', 1.0, 5.0, 0.12, 0.25);
}
document.addEventListener('DOMContentLoaded', updateAvoidDistance);

async function applyConfig(service, key, inputId) {
    const input = document.getElementById(inputId);
    if (!input) return;
    const val = parseFloat(input.value);
    if (isNaN(val)) {
        logConsole(`[ERR] Valeur invalide saisie pour ${service}.${key}`, 'error');
        return;
    }

    logConsole(`[TX] Envoi Config : ${service}.${key} = ${val} (Validation CRC32)`, 'sent');

    try {
        const resp = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ service, key, value: val })
        });

        if (resp.ok) {
            const resJson = await resp.json();
            logConsole(`[RX] Ack serveur : ${resJson.command || 'OK'}`, 'received');
        } else {
            logConsole(`[ERR] HTTP ${resp.status} lors de l'envoi de ${key}`, 'error');
        }
    } catch (err) {
        logConsole(`[ERR] Erreur de communication : ${err.message}`, 'error');
    }
}

function applyCustomConfig() {
    const srvSelect = document.getElementById('cfg-custom-srv');
    const keyInput = document.getElementById('cfg-custom-key');
    const valInput = document.getElementById('cfg-custom-val');

    if (!srvSelect || !keyInput || !valInput) return;
    const srv = srvSelect.value;
    const key = keyInput.value.trim();
    const val = parseFloat(valInput.value);

    if (!key || isNaN(val)) {
        logConsole('[ERR] Veuillez renseigner une clé valide et une valeur numérique float', 'error');
        return;
    }

    applyConfig(srv, key, 'cfg-custom-val');
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

async function sendEmergencyStop(service = 'MOTORS') {
    logConsole(`[TX] ARRÊT D'URGENCE GLOBAL émis vers ${service} (CRC32)`, 'sent');
    try {
        const resp = await fetch('/api/command', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ service, action: 'emergency' })
        });
        if (resp.ok) {
            const resJson = await resp.json();
            logConsole(`[RX] Arrêt d'urgence appliqué avec succès: ${resJson.status}`, 'received');
        } else {
            logConsole(`[ERR] HTTP ${resp.status}`, 'error');
        }
    } catch (err) {
        logConsole(`[ERR] Échec: ${err.message}`, 'error');
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
    while (DOM.cmdLogs.children.length > 200) {
        DOM.cmdLogs.removeChild(DOM.cmdLogs.firstChild);
    }
    DOM.cmdLogs.scrollTop = DOM.cmdLogs.scrollHeight;
}
