extends Node3D

var NetworkIPAddrRegex: RegEx = RegEx.new()
@onready var network_client: NetworkClient = $NetworkClient
@onready var btn_connect: Button = $GridContainer/btn_Connect
@onready var lb_status: Label = $GridContainer/lb_ConnectionStatusPackets
@onready var le_ip: LineEdit = $GridContainer/le_IpAdress
@onready var btn_start_fsm: Button = $GridContainer/btn_StartFSM
@onready var btn_stop_fsm: Button = $GridContainer/btn_StopFSM

# --- Support du mode Headless pour Optuna ---
var is_optuna_mode: bool = false
var optuna_timeout: float = 35.0
var optuna_sim_time: float = 0.0
var optuna_started: bool = false
var optuna_start_delay: float = 0.3
var optuna_dist_traveled: float = 0.0
var optuna_prev_pos: Vector3 = Vector3.ZERO
var optuna_has_moved: bool = false
var optuna_finished: bool = false

@onready var picar: Node3D = get_node_or_null("PiCar")
@onready var ball: Node3D = get_node_or_null("PiCar/Bille")

func _ready() -> void:
	NetworkIPAddrRegex.compile(r'^((25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)(:\d+)?$')
	
	# Connecter les signaux réseau de NetworkClient
	if network_client:
		network_client.connected_to_server.connect(_on_network_connected)
		network_client.disconnected_from_server.connect(_on_network_disconnected)
		network_client.connection_error.connect(_on_network_error)

	# Détection des arguments de lancement CLI pour Optuna
	var args: PackedStringArray = OS.get_cmdline_user_args()
	if args.is_empty():
		args = OS.get_cmdline_args()
	for i in range(args.size()):
		if args[i] == "--optuna":
			is_optuna_mode = true
		elif args[i] == "--time-scale" and i + 1 < args.size():
			Engine.time_scale = float(args[i + 1])
		elif args[i] == "--timeout" and i + 1 < args.size():
			optuna_timeout = float(args[i + 1])

	# Configurer le CircuitParcours sur la collision_layer 2 (ruban optique)
	var circuit = get_node_or_null("CircuitParcours")
	if circuit:
		_set_collision_layer_recursive(circuit, 2)

	# Obstacles : nœuds "obstacle*" (instances de obstacle.blend). Le mesh importé n'a pas de collision :
	# on en génère une (enveloppe convexe, couche 1) pour que la voiture soit bloquée et que le capteur ultrason le voie.
	for node in get_children():
		if String(node.name).begins_with("obstacle"):
			_add_obstacle_collision(node)

	if is_optuna_mode:
		print("[Optuna Godot] Headless runner actif (TimeScale=", Engine.time_scale, ", Timeout=", optuna_timeout, "s)")
		if picar:
			optuna_prev_pos = picar.global_position
		# Connexion automatique au SimBridge TCP 127.0.0.1:9000
		network_client.connect_to_server("127.0.0.1", 9000)

func _add_obstacle_collision(node: Node) -> void:
	for mesh in node.find_children("*", "MeshInstance3D", true, false):
		var mi := mesh as MeshInstance3D
		var has_body := false
		for child in mi.get_children():
			if child is StaticBody3D:
				has_body = true
		if not has_body:
			mi.create_convex_collision()

func _set_collision_layer_recursive(node: Node, layer: int) -> void:
	if node is StaticBody3D:
		node.collision_layer = layer
		node.collision_mask = layer
	for child in node.get_children():
		_set_collision_layer_recursive(child, layer)

func _on_network_connected() -> void:
	btn_connect.disabled = false
	btn_connect.text = "Disconnect"
	lb_status.text = "Connected!"

func _on_network_disconnected() -> void:
	btn_connect.disabled = false
	btn_connect.text = "Connect"
	lb_status.text = "Disconnected"

func _on_network_error(msg: String) -> void:
	btn_connect.disabled = false
	btn_connect.text = "Connect"
	lb_status.text = "Error: " + msg

func _on_quit_pressed() -> void:
	if network_client:
		network_client.disconnect_from_server()
	get_tree().quit()

func _on_connect_pressed() -> void:
	if network_client.is_connected:
		network_client.disconnect_from_server()
	else:
		var ip_str = le_ip.text.strip_edges()
		if NetworkIPAddrRegex.search(ip_str):
			btn_connect.disabled = true
			lb_status.text = "Connecting..."
			var target_port: int = 9000
			var target_ip: String = ip_str
			if ip_str.find(":") != -1:
				var parts = ip_str.split(":")
				target_ip = parts[0]
				target_port = int(parts[1])
			network_client.connect_to_server(target_ip, target_port)
		else:
			lb_status.text = "Wrong IP Address!"

func _on_start_fsm_pressed() -> void:
	if network_client and network_client.is_connected:
		network_client.send_trigger_start()
		lb_status.text = "FSM Démarré !"
	else:
		lb_status.text = "Erreur: Non connecté !"

func _on_stop_fsm_pressed() -> void:
	if network_client and network_client.is_connected:
		network_client.send_emergency_stop()
		lb_status.text = "Arrêt d'urgence FSM"
	else:
		lb_status.text = "Erreur: Non connecté !"

# --- Boucle d'évaluation Headless pour Optuna ---
func _physics_process(delta: float) -> void:
	if not is_optuna_mode or optuna_finished:
		return

	if not network_client or not network_client.is_connected:
		return

	# Attente brève de stabilisation avant envoi du trigger_start
	if not optuna_started:
		optuna_start_delay -= delta
		if optuna_start_delay <= 0.0:
			optuna_started = true
			network_client.send_trigger_start()
			print("[Optuna Godot] Top départ envoyé au FSM !")
		return

	optuna_sim_time += delta

	# Suivi de la distance réelle parcourue
	if picar:
		var cur_pos: Vector3 = picar.global_position
		var step_dist: float = cur_pos.distance_to(optuna_prev_pos)
		optuna_dist_traveled += step_dist
		optuna_prev_pos = cur_pos
		if optuna_dist_traveled > 0.2:
			optuna_has_moved = true

	var max_accel: float = 0.0
	if ball and "max_accel_observed" in ball:
		max_accel = float(ball.max_accel_observed)

	# 1. Condition d'échec : bille tombée hors de la cuvette
	if ball and bool(ball.get("is_dropped")):
		_finish_optuna("ball_dropped", optuna_sim_time, max_accel, true, optuna_dist_traveled)
		return

	# 2. Condition de succès : fin de parcours au T (vitesse redescendue à 0 après avoir parcouru le circuit)
	var speed: float = absf(float(network_client.data_received.get("velocity", 0.0)))
	if optuna_has_moved and optuna_dist_traveled >= 3.8 and speed < 0.02 and optuna_sim_time > 4.0:
		_finish_optuna("success", optuna_sim_time, max_accel, false, optuna_dist_traveled)
		return

	# 3. Timeout : temps limite de simulation atteint
	if optuna_sim_time >= optuna_timeout:
		_finish_optuna("timeout", optuna_sim_time, max_accel, false, optuna_dist_traveled)
		return

func _finish_optuna(status: String, lap_time: float, max_accel: float, ball_dropped: bool, dist: float) -> void:
	optuna_finished = true
	var result: Dictionary = {
		"status": status,
		"lap_time": snappedf(lap_time, 0.01),
		"max_accel": snappedf(max_accel, 0.001),
		"ball_dropped": ball_dropped,
		"distance": snappedf(dist, 0.01)
	}
	print("\nOPTUNA_RESULT:" + JSON.stringify(result) + "\n")
	if network_client:
		network_client.disconnect_from_server()
	get_tree().quit(0 if status == "success" else 1)
