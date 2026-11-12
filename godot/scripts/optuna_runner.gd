extends RefCounted

## Mode d'évaluation automatique pour l'optimiseur (tools/optimizer/optimize_godot.py).
##
## Godot est lancé sans fenêtre avec : -- --optuna --time-scale <N> --timeout <secondes>
## Le runner envoie l'ordre de départ, suit le tour et imprime UNE ligne "OPTUNA_RESULT:{json}" quand c'est fini,
## puis quitte avec le code 0 (succès) ou 1 (échec ou délai dépassé). Le script Python lit cette ligne.
##
## Le tour s'arrête sur le premier de ces trois cas :
##   ball_dropped : la bille est tombée ;
##   success      : le robot a parcouru au moins 3,8 m, puis s'est arrêté à la fin du parcours ;
##   timeout      : le temps de simulation dépasse la limite.

const START_DELAY: float = 0.3         # attente avant l'ordre de départ, pour laisser la connexion se stabiliser (s)
const FINISH_DISTANCE: float = 3.8     # distance minimale pour compter un tour réussi (m)
const FINISH_SPEED: float = 0.02       # sous cette vitesse, le robot est considéré arrêté (m/s)
const FINISH_MIN_TIME: float = 4.0     # un succès avant ce temps n'est pas crédible (s)

var _host: Node                        # le nœud principal : sert à quitter l'application
var _network: NetworkClient
var _picar: Node3D
var _ball: Node
var _timeout: float
var _start_delay: float = START_DELAY
var _started: bool = false
var _finished: bool = false
var _sim_time: float = 0.0
var _distance: float = 0.0
var _prev_pos: Vector3

func _init(host: Node, network: NetworkClient, picar: Node3D, ball: Node, timeout: float) -> void:
	_host = host
	_network = network
	_picar = picar
	_ball = ball
	_timeout = timeout
	if picar:
		_prev_pos = picar.global_position
	print("[Optuna Godot] Headless runner actif (TimeScale=", Engine.time_scale, ", Timeout=", timeout, "s)")
	_network.connect_to_server("127.0.0.1", 9000)

## À appeler à chaque pas de physique
func step(delta: float) -> void:
	if _finished or not _network.is_connected:
		return

	# Courte attente de stabilisation avant l'ordre de départ
	if not _started:
		_start_delay -= delta
		if _start_delay <= 0.0:
			_started = true
			_network.send_trigger_start()
			print("[Optuna Godot] Top départ envoyé au FSM !")
		return

	_sim_time += delta
	if _picar:
		var pos: Vector3 = _picar.global_position
		_distance += pos.distance_to(_prev_pos)
		_prev_pos = pos

	if _ball and _ball.is_dropped:
		_finish("ball_dropped", true)
	elif _distance >= FINISH_DISTANCE and _sim_time > FINISH_MIN_TIME \
			and absf(float(_network.data_received.get("velocity", 0.0))) < FINISH_SPEED:
		_finish("success", false)
	elif _sim_time >= _timeout:
		_finish("timeout", false)

func _finish(status: String, ball_dropped: bool) -> void:
	_finished = true
	var result: Dictionary = {
		"status": status,
		"lap_time": snappedf(_sim_time, 0.01),
		"max_accel": snappedf(_ball.max_accel_observed if _ball else 0.0, 0.001),
		"ball_dropped": ball_dropped,
		"distance": snappedf(_distance, 0.01),
	}
	print("\nOPTUNA_RESULT:" + JSON.stringify(result) + "\n")
	_network.disconnect_from_server()
	_host.get_tree().quit(0 if status == "success" else 1)
