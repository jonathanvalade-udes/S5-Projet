extends Node3D

# Cinématique métrique unifiée S10 pour PiCar-S (Godot 4)
# Empattement L = 0.14 m, angle de braquage max = 48°

var ray: RayCast3D = null
var distance: float = 100.0
var speed: float = 0.0
var steer_dir: float = 0.0

var lfs: Node = null
var network: NetworkClient = null

# Animation des roues : roulement de chaque roue + braquage des roues avant.
# - Les 4 roues sont des nœuds "Roue*" (instances de Roue.blend), enfants de PiCar. Leur axe est leur axe X local.
# - Les petits axes de roue (dans robot.blend) ne roulent pas : seuls ceux de l'avant suivent le braquage.
# Tout est calculé dans le repère de la voiture : l'avant est -X, le haut est +Y, l'axe des roues est Z.
const MAX_STEER_DEG: float = 48.0  # même valeur que la cinématique d'Ackermann plus bas

var model: Node3D = null
var parts: Array[Node3D] = []             # roues puis petits axes
var part_rest: Array[Transform3D] = []    # pose de repos, en repère de la voiture
var part_pivot: Array[Vector3] = []       # pivot du braquage, en repère de la voiture
var part_spin: Array[float] = []          # +1 / -1 : sens de roulement ; 0 : ne roule pas (axe)
var part_front: Array[bool] = []          # true : suit le braquage
var wheel_radius_m: float = 0.0456        # rayon de contact au sol, en mètres (recalculé depuis le modèle)
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

# Collision : la voiture est déplacée à la main (ce n'est pas un CharacterBody3D). Avant chaque mouvement, on
# teste la boîte qu'elle occuperait ; si elle entre dans un obstacle (couche 1), le mouvement est annulé.
const OBSTACLE_MASK: int = 1
var _body_shape := BoxShape3D.new()
var _body_center := Vector3.ZERO          # centre de la boîte, en repère de la voiture
var _has_body_box: bool = false
var _own_bodies: Array[RID] = []          # corps de la voiture elle-même (bille, plaquette) : ignorés

func _ready() -> void:
	Realism.setup()
	_next_kick = Realism.rng.randf_range(0.5, 1.5) * Realism.p["kick_interval_s"]
	ray = get_node_or_null("RayCast3D")
	var lfs_node = get_node_or_null("line_follower_sensor")
	if lfs_node and lfs_node.get_child_count() > 0:
		lfs = lfs_node.get_child(0)
	network = get_node_or_null("/root/Main/NetworkClient")
	_find_wheels()
	_build_collision_box()

# Repère les roues ("Roue*", enfants de PiCar) et les petits axes de roue du modèle (pièces de 0,5 à 0,8 unités)
func _find_wheels() -> void:
	var to_car: Transform3D = global_transform.affine_inverse()
	var wheel_idx: Array[int] = []
	for child in get_children():
		if not (child is Node3D) or not String(child.name).begins_with("Roue"):
			continue
		var rest: Transform3D = to_car * (child as Node3D).global_transform
		# Le haut de la roue doit aller vers l'avant (-X) : rotation autour de +Z, donc on suit le sens de l'axe.
		var spin: float = 1.0 if rest.basis.x.z >= 0.0 else -1.0
		for mesh in child.find_children("*", "MeshInstance3D", true, false):
			var mi := mesh as MeshInstance3D
			wheel_radius_m = ((to_car * mi.global_transform) * mi.get_aabb()).size.y / 2.0
			break
		wheel_idx.append(parts.size())
		_add_part(child as Node3D, rest, rest.origin, spin)

	model = get_node_or_null("RobotModel") as Node3D
	if model != null:
		var to_model: Transform3D = model.global_transform.affine_inverse()
		for n in model.find_children("*", "MeshInstance3D", true, false):
			var mi := n as MeshInstance3D
			var box: AABB = (to_model * mi.global_transform) * mi.get_aabb()
			var largest: float = maxf(box.size.x, maxf(box.size.y, box.size.z))
			if largest > 0.5 and largest <= 0.8:
				var rest: Transform3D = to_car * mi.global_transform
				var center: Vector3 = rest * mi.get_aabb().get_center()
				# Un petit axe braque autour du centre de la roue la plus proche, pas autour de lui-même
				var pivot: Vector3 = center
				var best_d: float = INF
				for w in wheel_idx:
					var d: float = absf(part_pivot[w].x - center.x) + absf(part_pivot[w].z - center.z)
					if d < best_d:
						best_d = d
						pivot = part_pivot[w]
				_add_part(mi, rest, pivot, 0.0)

	# Avant = pièces dont le pivot est plus en avant (-X) que le milieu des roues
	var x_mid: float = 0.0
	for w in wheel_idx:
		x_mid += part_pivot[w].x
	x_mid /= maxf(wheel_idx.size(), 1)
	for i in parts.size():
		part_front[i] = part_pivot[i].x < x_mid
	print("[PiCar] pièces animées : ", parts.size(), " dont ", wheel_idx.size(), " roues (rayon ", wheel_radius_m, " m)")

func _add_part(node: Node3D, rest: Transform3D, pivot: Vector3, spin: float) -> void:
	parts.append(node)
	part_rest.append(rest)
	part_pivot.append(pivot)
	part_spin.append(spin)
	part_front.append(false)

# distance : mètres parcourus (positif = avant) ; steer_rad : angle de braquage demandé (rad)
func _animate_wheels(distance: float, steer_rad: float, delta: float) -> void:
	if parts.is_empty():
		return
	if wheel_radius_m > 0.0:
		wheel_angle = fposmod(wheel_angle + distance / wheel_radius_m, TAU)
	steer_visual = lerpf(steer_visual, steer_rad, clampf(delta * 15.0, 0.0, 1.0))
	var steer_basis := Basis(Vector3.UP, steer_visual)  # + = même sens que rotate_y de la voiture
	var car: Transform3D = global_transform
	for i in parts.size():
		var local := part_rest[i]
		if part_spin[i] != 0.0:
			local = local * Transform3D(Basis(Vector3.RIGHT, wheel_angle * part_spin[i]), Vector3.ZERO)  # axe X de la roue
		if part_front[i]:
			var p: Vector3 = part_pivot[i]
			local = Transform3D(steer_basis, p - steer_basis * p) * local
		parts[i].global_transform = car * local

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

# Boîte de collision = encombrement du châssis et des roues (sans la plaquette ni la bille)
func _build_collision_box() -> void:
	var to_car: Transform3D = global_transform.affine_inverse()
	var meshes: Array = []
	if model != null:
		meshes.append_array(model.find_children("*", "MeshInstance3D", true, false))
	for child in get_children():
		if String(child.name).begins_with("Roue"):
			meshes.append_array(child.find_children("*", "MeshInstance3D", true, false))
	var box := AABB()
	var first := true
	for n in meshes:
		var mi := n as MeshInstance3D
		var b: AABB = (to_car * mi.global_transform) * mi.get_aabb()
		box = b if first else box.merge(b)
		first = false
	if first:
		return
	_body_shape.size = box.size
	_has_body_box = true
	_body_center = box.get_center()
	for body in find_children("*", "CollisionObject3D", true, false):
		_own_bodies.append((body as CollisionObject3D).get_rid())

# true si la voiture entrerait dans un obstacle en faisant ce mouvement (avance `step`, puis rotation `yaw`)
func _would_collide(step: Vector3, yaw: float) -> bool:
	if not _has_body_box:
		return false
	var t: Transform3D = global_transform
	t.origin += t.basis * step
	t.basis = Basis(Vector3.UP, yaw) * t.basis
	var params := PhysicsShapeQueryParameters3D.new()
	params.shape = _body_shape
	params.transform = t * Transform3D(Basis.IDENTITY, _body_center)
	params.collision_mask = OBSTACLE_MASK
	params.exclude = _own_bodies
	return not get_world_3d().direct_space_state.intersect_shape(params, 1).is_empty()

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
	var step := Vector3(-actual_speed * delta, 0, 0)
	var moved: float = actual_speed * delta
	if _would_collide(step, angular_velocity * delta):
		moved = 0.0  # bloquée par un obstacle : la voiture ne bouge pas (les roues ne roulent pas non plus)
	else:
		translate(step)
		rotate_y(angular_velocity * delta)
		_apply_kick(delta)
	# Roues : roulement + braquage des roues avant (on voit donc les roues désalignées)
	_animate_wheels(moved, actual_steer_rad, delta)

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
