// Vérifie que le catalogue des réglages de l'interface web (PARAMS dans web/app.js) colle au code du robot :
//   - chaque réglage est bien accepté par son service (la clé existe dans le code C++) ;
//   - chaque valeur « actuelle » (live) existe dans le JSON de télémétrie produit par ProtocolHandler.cpp ;
//   - les valeurs par défaut sont celles de fsm.conf ;
//   - les limites affichées (hard) sont celles que le robot applique vraiment ;
//   - le catalogue est cohérent (pas de doublon, défaut dans les limites, groupes existants).
// Lancement : node tools/telemetryClient/tests/test_params_catalogue.js   (depuis la racine du dépôt)

const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..', '..');
const read = (rel) => fs.readFileSync(path.join(ROOT, rel), 'utf8');

let failures = 0;
function check(ok, message) {
    if (!ok) {
        failures++;
        console.error('  ÉCHEC : ' + message);
    }
}

// --- Charge app.js avec un faux navigateur, juste pour lire PARAMS et PARAM_GROUPS ---
const sandbox = {
    window: { addEventListener() {} },
    document: { getElementById: () => null, createElement: () => ({}) },
    EventSource: function () {},
    console,
};
vm.createContext(sandbox);
vm.runInContext(read('tools/telemetryClient/web/app.js'), sandbox);
const PARAMS = vm.runInContext('PARAMS', sandbox);
const GROUPS = vm.runInContext('PARAM_GROUPS', sandbox);

// --- Sources du robot ---
const serviceSource = {
    FSM: read('services/FSM/src/FSM.cpp') + read('services/FSM/inc/AvoidanceSettings.hpp') + read('services/FSM/inc/ObstacleTuning.hpp'),
    MOTORS: read('services/Motors/src/Motors.cpp'),
    SENSORS: read('services/Sensors/src/Sensors.cpp'),
};
const protocol = read('tools/telemetryClient/src/ProtocolHandler.cpp');
const fsmConf = {};
for (const line of read('fsm.conf').split(/\r?\n/)) {
    const m = line.match(/^([A-Z0-9_]+)=([-0-9.eE]+)/);
    if (m) fsmConf[m[1]] = parseFloat(m[2]);
}

// Champs de télémétrie de chaque section « config » : du marqueur jusqu'à la fin du bloc
function configFields(root) {
    const start = protocol.indexOf(`"\\"${root}\\":{"`);
    const cfg = protocol.indexOf('"\\"config\\":{"', start);
    const end = protocol.indexOf('ss << "},";', cfg);
    const names = new Set();
    for (const m of protocol.slice(cfg, end).matchAll(/\\"([a-z0-9_]+)\\":/g)) names.add(m[1]);
    return names;
}
const telemetryFields = { fsm: configFields('fsm'), sensors: configFields('sensors'), motors: configFields('motors') };
const avoidanceFields = new Set(
    [...protocol.slice(protocol.indexOf('\\"avoidance\\"'), protocol.indexOf('ss << "},";', protocol.indexOf('\\"avoidance\\"')))
        .matchAll(/\\"([a-z0-9_]+)\\":/g)].map((m) => m[1]));

// Limites réellement appliquées par le robot, lues dans le code
function tableLimits(header, key) {
    const m = header.match(new RegExp(`\\{"${key}",\\s*&\\w+::\\w+,\\s*([-0-9.]+)f?,\\s*([-0-9.]+)f?\\}`));
    return m ? [parseFloat(m[1]), parseFloat(m[2])] : null;
}
const avoidHeader = read('services/FSM/inc/AvoidanceSettings.hpp');
const tuningHeader = read('services/FSM/inc/ObstacleTuning.hpp');
const navHeader = read('services/FSM/inc/FsmNavigator.hpp');
const constant = (name) => parseFloat(navHeader.match(new RegExp(`${name} = ([-0-9.]+)f`))[1]);
const fsmCpp = read('services/FSM/src/FSM.cpp');
const sensorsCpp = read('services/Sensors/src/Sensors.cpp');
const robotLimits = {
    'FSM.MAX_POWER_PCT': [constant('MIN_POWER_PCT'), constant('MAX_POWER_PCT')],
    'FSM.STRAIGHT_CRUISE_SPEED': [constant('MIN_CRUISE_SPEED'), constant('MAX_CRUISE_SPEED')],
    'FSM.CURVE_SAFETY_FACTOR': (() => { const m = fsmCpp.match(/m_curve_safety_factor = std::clamp\(value, ([0-9.]+)f, ([0-9.]+)f\)/); return m && [parseFloat(m[1]), parseFloat(m[2])]; })(),
    'FSM.HZ': (() => { const m = fsmCpp.match(/m_hz = std::clamp\(static_cast<int>\(value\), (\d+), (\d+)\)/); return m && [parseInt(m[1]), parseInt(m[2])]; })(),
    'SENSORS.FILTER_ALPHA': (() => { const m = sensorsCpp.match(/m_filter_alpha = std::clamp\(value, ([0-9.]+)f, ([0-9.]+)f\)/); return m && [parseFloat(m[1]), parseFloat(m[2])]; })(),
    'SENSORS.MEDIAN_WINDOW': [1, 15],
};
for (const p of PARAMS) {
    const fromTable = tableLimits(avoidHeader, p.key) || tableLimits(tuningHeader, p.key);
    if (fromTable) robotLimits[`${p.service}.${p.key}`] = fromTable;
}
const stopMin = fsmCpp.match(/m_obstacle_stop_dist_cm = std::max\(value, ([0-9.]+)f\)/);
if (stopMin) robotLimits['FSM.OBSTACLE_STOP_DIST_CM'] = [parseFloat(stopMin[1]), p => p.hard[1]];

// --- Vérifications ---
console.log(`Catalogue : ${PARAMS.length} réglages dans ${GROUPS.length} groupes`);
const seen = new Set();
const groupIds = new Set(GROUPS.map((g) => g.id));
for (const p of PARAMS) {
    const id = `${p.service}.${p.key}`;
    check(!seen.has(id), `doublon : ${id}`);
    seen.add(id);
    check(groupIds.has(p.group), `${id} : groupe inconnu "${p.group}"`);
    check(typeof p.title === 'string' && p.title.length > 0 && typeof p.desc === 'string' && p.desc.length > 20, `${id} : titre ou description manquant`);
    check(p.options || p.step > 0, `${id} : pas (step) manquant`);

    // Accepté par le service
    const src = serviceSource[p.service];
    check(src && src.includes(`"${p.key}"`), `${id} : la clé n'existe pas dans le code du service ${p.service}`);

    // Cohérence interne
    if (p.hard) check(p.def >= p.hard[0] && p.def <= p.hard[1], `${id} : défaut ${p.def} hors des limites ${p.hard}`);
    if (p.rec) {
        check(p.def >= p.rec[0] && p.def <= p.rec[1], `${id} : défaut ${p.def} hors de la plage conseillée ${p.rec}`);
        if (p.hard) check(p.rec[0] >= p.hard[0] && p.rec[1] <= p.hard[1], `${id} : plage conseillée ${p.rec} dépasse les limites ${p.hard}`);
    }

    // Valeur actuelle présente dans la télémétrie
    if (p.live) {
        const [root, cfg, ...rest] = p.live.split('.');
        const field = rest[rest.length - 1];
        const pool = rest.length === 2 ? avoidanceFields : telemetryFields[root];
        check(cfg === 'config' && pool && pool.has(field), `${id} : "${p.live}" n'existe pas dans le JSON de ProtocolHandler.cpp`);
    }

    // Défaut = fsm.conf
    if (p.service === 'FSM' && p.key in fsmConf) {
        check(Math.abs(fsmConf[p.key] - p.def) < 1e-6, `${id} : défaut ${p.def} différent de fsm.conf (${fsmConf[p.key]})`);
    }

    // Limites affichées = limites du robot
    const limit = robotLimits[id];
    if (limit) {
        check(p.hard, `${id} : le robot limite à ${limit} mais le catalogue n'affiche aucune limite`);
        if (p.hard) {
            const upper = typeof limit[1] === 'function' ? p.hard[1] : limit[1];
            check(Math.abs(p.hard[0] - limit[0]) < 1e-6 && Math.abs(p.hard[1] - upper) < 1e-6, `${id} : limites affichées ${p.hard} ≠ limites du robot ${limit}`);
        }
    } else if (p.hard && p.service === 'FSM') {
        check(false, `${id} : des limites sont affichées (${p.hard}) mais je n'en trouve aucune dans le code du robot`);
    }
    if (p.dist && p.dist.speedKey) check(PARAMS.some((q) => q.key === p.dist.speedKey), `${id} : speedKey ${p.dist.speedKey} introuvable`);
}

// Toute clé que le FSM accepte (hors celles volontairement absentes) doit être dans l'interface
const apply = fsmCpp.slice(fsmCpp.indexOf('bool FSM::apply_config_key'));
const acceptedByFsm = [...apply.matchAll(/key == "([A-Z0-9_]+)"/g)].map((m) => m[1]);
const notInUi = { REVERSE_SPEED: 'sans effet depuis la manœuvre en 12 étapes' };
for (const key of acceptedByFsm) {
    if (!(key in notInUi)) check(PARAMS.some((p) => p.service === 'FSM' && p.key === key), `FSM.${key} est accepté par le FSM mais absent de l'interface`);
}
for (const key of [...avoidHeader.matchAll(/\{"(AVOID_[A-Z0-9_]+)"/g), ...tuningHeader.matchAll(/\{"(OBSTACLE_[A-Z0-9_]+)"/g)].map((m) => m[1])) {
    check(PARAMS.some((p) => p.key === key), `${key} est accepté par le FSM mais absent de l'interface`);
}
for (const key of ['AUTO_START_DELAY', 'REVERSE_SPEED']) {
    check(!PARAMS.some((p) => p.key === key), `${key} ne devrait plus être dans l'interface (${key === 'REVERSE_SPEED' ? notInUi[key] : "le FSM ne l'accepte pas"})`);
}

if (failures > 0) {
    console.error(`\n${failures} échec(s)`);
    process.exit(1);
}
console.log('test_params_catalogue : tous les tests passent');
