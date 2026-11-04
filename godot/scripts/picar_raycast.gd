extends Node3D

# Cinématique métrique unifiée S10 pour PiCar-S (Godot 4)
# Empattement L = 0.14 m, angle de braquage max = 48°

var ray: RayCast3D = null
var distance: float = 100.0
var speed: float = 0.0
var steer_dir: float = 0.0

var lfs: Node = null
var network: NetworkClient = null

# Animation des roues : rotation (roulement) de chaque pneu + braquage des roues avant.
# - Roulement : chaque pneu tourne autour de son axe de cylindre (Y local), sans déformation
#   même si le modèle est étiré à l'import.
# - Braquage : les pneus avant (et leurs petits axes) pivotent autour de l'axe vertical du modèle.
# Repère du modèle : l'avant du robot est +Z, le haut est +Y.
const MAX_STEER_DEG: float = 48.0  # même valeur que la cinématique d'Ackermann plus bas

var model: Node3D = null
var parts: Array[MeshInstance3D] = []
var part_rest: Array[Transform3D] = []    # pose de repos, en repère du modèle
var part_center: Array[Vector3] = []      # centre du mesh, en repère du mesh
var part_pivot: Array[Vector3] = []       # pivot du braquage, en repère du modèle
var part_spin: Array[float] = []          # +1 / -1 : sens de roulement ; 0 : ne roule pas (axe)
var part_front: Array[bool] = []          # true : suit le braquage
var wheel_radius_m: float = 0.055         # rayon de contact au sol, en mètres
var wheel_angle: float = 0.0
var steer_visual: float = 0.0             # angle de braquage affiché (rad), lissé

# Mouvement RÉEL de la voiture, avec les imperfections de Realism (lu aussi par la bille)
var actual_speed: float = 0.0
var actual_steer_rad: float = 0.0
var _servo_play: float = 0.0              # position du servo avec son jeu (rad)
var _servo_rad: float = 0.0               # angle réel des roues, après limite de vitesse du servo (rad)
var _speed_noise: float = 0.0             # variation lente de la vitesse (fraction)
var _steer_noise: float = 0.0             # dérive lente de l'angle de braquage (degrés)
var _cmd_queue: Array = []                # ordres reçus [temps, vitesse, direction], appliqués avec un délai
var _clock: float = 0.0
var _next_kick: float = 0.0               # secondes avant le prochain petit choc

func _ready() -> void:
	Realism.setup()
	_next_kick = Realism.rng.randf_range(0.5, 1.5) * Realism.p["kick_interval_s"]
	ray = get_node_or_null("RayCast3D")
	var lfs_node = get_node_or_null("line_follower_sensor")
	if lfs_node and lfs_node.get_child_count() > 0:
		lfs = lfs_node.get_child(0)
	network = get_node_or_null("/root/Main/NetworkClient")
	_find_wheels()

# Repère les pièces mobiles d'après leur taille (en unités du modèle) :
#   pneus : 1,4 à 3,6   |   petits axes de roue : 0,5 à 0,8   |   (le capteur ~0,4 et le châssis ~4,4 sont ignorés)
func _find_wheels() -> void:
	model = get_node_or_null("RobotModel") as Node3D
	if model == null:
		print("[PiCar] ERREUR : nœud RobotModel introuvable sous PiCar")
		return
	var inv: Transform3D = model.global_transform.affine_inverse()
	var tyres: Array[int] = []
	var zs: Array[float] = []
	for n in model.find_children("*", "MeshInstance3D", true, false):
		var mi := n as MeshInstance3D
		var to_model: Transform3D = inv * mi.global_transform
		var box: AABB = to_model * mi.get_aabb()
		var largest: float = maxf(box.size.x, maxf(box.size.y, box.size.z))
		var is_tyre: bool = largest > 1.4 and largest < 3.6
		var is_stub: bool = largest > 0.5 and largest <= 0.8
		if not (is_tyre or is_stub):
			continue
		var spin: float = 0.0
		if is_tyre:
			# Le haut du pneu doit aller vers l'avant (+Z) : rotation positive autour de +X du modèle.
			var axis_in_model: Vector3 = to_model.basis * Vector3.UP
			spin = 1.0 if axis_in_model.x >= 0.0 else -1.0
			wheel_radius_m = box.size.y / 2.0 * model.global_transform.basis.y.length()
			tyres.append(parts.size())
		parts.append(mi)
		part_rest.append(to_model)
		part_center.append(mi.get_aabb().get_center())
		part_pivot.append(to_model * mi.get_aabb().get_center())
		part_spin.append(spin)
		zs.append(box.get_center().z)
		part_front.append(false)
	# Pièces à l'avant = celles dont le centre est plus en avant que le milieu du robot
	var z_mid: float = 0.0
	for z in zs: z_mid += z
	z_mid /= maxf(zs.size(), 1)
	for i in parts.size():
		part_front[i] = zs[i] > z_mid
	# Un petit axe pivote autour du centre du pneu avant le plus proche (même côté), pas autour de lui-même
	for i in parts.size():
		if part_spin[i] != 0.0 or not part_front[i]:
			continue
		var best: int = -1
		var best_d: float = INF
		for t in tyres:
			var d: float = absf(part_pivot[t].x - part_pivot[i].x) + absf(part_pivot[t].z - part_pivot[i].z)
			if d < best_d:
				best_d = d
				best = t
		if best >= 0:
			part_pivot[i] = part_pivot[best]
	print("[PiCar] pièces animées : ", parts.size(), " dont ", tyres.size(), " pneus (rayon ", wheel_radius_m, " m)")

# distance : mètres parcourus (positif = avant) ; steer_rad : angle de braquage demandé (rad)
func _animate_wheels(distance: float, steer_rad: float, delta: float) -> void:
	if parts.is_empty():
		return
	if wheel_radius_m > 0.0:
		wheel_angle = fposmod(wheel_angle + distance / wheel_radius_m, TAU)
	steer_visual = lerpf(steer_visual, steer_rad, clampf(delta * 15.0, 0.0, 1.0))
	var steer_basis := Basis(Vector3.UP, steer_visual)  # + = vers +X du modèle, comme rotate_y du robot
	var m: Transform3D = model.global_transform
	for i in parts.size():
		var local := part_rest[i]
		if part_spin[i] != 0.0:
			var rot := Basis(Vector3.UP, wheel_angle * part_spin[i])
			var c: Vector3 = part_center[i]
			local = local * Transform3D(rot, c - rot * c)
		if part_front[i]:
			var p: Vector3 = part_pivot[i]
			local = Transform3D(steer_basis, p - steer_basis * p) * local
		parts[i].global_transform = m * local

func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_R:
		Realism.toggle()

# Retarde les ordres de Python (latence de la communication). Renvoie (vitesse, direction).
func _apply_latency(cmd_speed: float, cmd_dir: float, delta: float) -> Vector2:
	_clock += delta
	if not Realism.enabled:
		_cmd_queue.clear()
		return Vector2(cmd_speed, cmd_dir)
	_cmd_queue.append([_clock, cmd_speed, cmd_dir])
	var due: float = _clock - Realism.p["latency_s"]
	# on garde l'ordre le plus récent qui date d'au moins `latency_s`
	while _cmd_queue.size() > 1 and _cmd_queue[1][0] <= due:
		_cmd_queue.pop_front()
	return Vector2(_cmd_queue[0][1], _cmd_queue[0][2])

# Calcule la vitesse et l'angle de braquage RÉELS à partir des ordres (actual_speed, actual_steer_rad)
func _update_actual_motion(cmd_speed: float, cmd_dir: float, delta: float) -> void:
	var max_steer: float = deg_to_rad(MAX_STEER_DEG)
	if not Realism.enabled:
		actual_speed = cmd_speed
		actual_steer_rad = cmd_dir * max_steer
		_servo_play = actual_steer_rad
		_servo_rad = actual_steer_rad
		return
	# Direction : biais d'alignement + dérive lente, puis jeu du servo et vitesse limitée du servo.
	# La dérive est un bruit lissé (constante de temps 0,4 s) : un bruit tiré à chaque image ferait
	# trembler les roues à gauche et à droite, ce qui n'est pas ce que fait un vrai servo.
	const STEER_NOISE_TAU: float = 0.4
	_steer_noise += -_steer_noise / STEER_NOISE_TAU * delta \
			+ Realism.noise("steer_noise_deg") * sqrt(2.0 * delta / STEER_NOISE_TAU) * Realism.gauss(1.0)
	var target: float = cmd_dir * max_steer + deg_to_rad(Realism.p["steer_bias_deg"] + _steer_noise)
	var half_play: float = deg_to_rad(Realism.p["steer_backlash_deg"]) / 2.0
	if target > _servo_play + half_play:
		_servo_play = target - half_play
	elif target < _servo_play - half_play:
		_servo_play = target + half_play
	_servo_rad = move_toward(_servo_rad, _servo_play, deg_to_rad(Realism.p["servo_speed_dps"]) * delta)
	actual_steer_rad = clampf(_servo_rad, -max_steer, max_steer)
	# Vitesse : gain moteur et variation lente (processus d'Ornstein-Uhlenbeck, constante de temps 1 s)
	_speed_noise += -_speed_noise * delta + Realism.noise("motor_noise") * sqrt(2.0 * delta) * Realism.gauss(1.0)
	actual_speed = cmd_speed * Realism.p["motor_gain"] * (1.0 + _speed_noise)

# Petit choc aléatoire (bosse, gravier) : fait pivoter un peu la voiture
func _apply_kick(delta: float) -> void:
	if not Realism.enabled or absf(actual_speed) < 0.05:
		return
	_next_kick -= delta
	if _next_kick > 0.0:
		return
	_next_kick = Realism.rng.randf_range(0.5, 1.5) * Realism.p["kick_interval_s"]
	rotate_y(deg_to_rad(Realism.rng.randf_range(-1.0, 1.0) * Realism.noise("kick_yaw_deg")))

func _process(delta: float) -> void:
	if network == null:
		network = get_node_or_null("/root/Main/NetworkClient")

	# Si pas connecté au serveur WebSocket Python, la voiture RESTE STRICTEMENT IMMOBILE
	if network == null or not network.is_connected:
		speed = 0.0
		steer_dir = 0.0
		actual_speed = 0.0
		actual_steer_rad = 0.0
		return

	# Mode connecté : cinématique pilotée exclusivement par Python
	var rx_speed: float = float(network.data_received.get("velocity", 0.0))
	var rx_steer: float = float(network.data_received.get("direction", 0.0))
	var cmd: Vector2 = _apply_latency(rx_speed, rx_steer, delta)
	_update_actual_motion(cmd.x, cmd.y, delta)

	# Cinématique métrique d'Ackermann, avec le mouvement réel
	var angular_velocity: float = (actual_speed / 0.14) * tan(actual_steer_rad)
	translate(Vector3(-actual_speed * delta, 0, 0))
	rotate_y(angular_velocity * delta)
	_apply_kick(delta)
	# Roues : roulement + braquage des roues avant (on voit donc les roues désalignées)
	_animate_wheels(actual_speed * delta, actual_steer_rad, delta)

	# Feedback télémesure vers Python
	network.data_to_send["velocity"] = rx_speed
	if "direction" in network.data_to_send:
		network.data_to_send.erase("direction")

func _physics_process(_delta: float) -> void:
	if ray and ray.is_colliding():
		var col_pt = ray.get_collision_point()
		distance = col_pt.distance_to(position)
	else:
		distance = 1.0  # 1.0 mètre

	if network and network.is_connected:
		# Distance ultrason transmise en centimètres
		network.data_to_send["distance"] = round(Realism.noisy_distance(distance) * 100.0)
