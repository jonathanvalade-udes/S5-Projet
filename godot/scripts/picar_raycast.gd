extends Node3D

## Le PiCar : mouvement, collisions et capteur ultrason.
##
## À chaque image, la voiture lit les commandes du FSM (vitesse et braquage, reçues par NetworkClient) et avance
## selon un modèle d'Ackermann : seules les roues avant tournent, comme un vélo. Sans connexion, elle reste immobile.
##   vitesse angulaire = (vitesse / empattement) * tan(braquage)
## Le mouvement RÉEL (actual_speed, actual_steer_rad, actual_yaw_rate) peut différer des commandes à cause du mode
## réaliste (realism.gd) : c'est lui que lit la bille (ball_monitor.gd).
##
## Le PiCar n'est pas un corps physique : il est déplacé à la main. Avant chaque pas, on teste s'il entrerait dans un
## obstacle (couche 1) et, si oui, on annule le pas.

const WheelAnimator = preload("res://scripts/wheel_animator.gd")

const WHEELBASE: float = 0.14        # empattement (m)
const MAX_STEER_DEG: float = 48.0    # braquage maximal des roues (la commande -1..+1 le multiplie)
const OBSTACLE_MASK: int = 1         # couche de collision des obstacles
const NO_ECHO_DISTANCE: float = 1.0  # distance renvoyée quand le rayon ultrason ne touche rien (m)

## Mouvement réel de la voiture, après les imperfections du mode réaliste
var actual_speed: float = 0.0       # m/s
var actual_steer_rad: float = 0.0   # rad
var actual_yaw_rate: float = 0.0    # rad/s

var network: NetworkClient = null

var _ray: RayCast3D = null          # rayon de l'ultrason, dirigé vers l'avant
var _wheels: WheelAnimator = null   # animation des roues

# Imperfections (mode réaliste)
var _servo_play: float = 0.0        # position du servo avec son jeu (rad)
var _servo_rad: float = 0.0         # angle réel des roues, après limite de vitesse du servo (rad)
var _speed_noise: float = 0.0       # variation lente de la vitesse (fraction)
var _steer_noise: float = 0.0       # dérive lente du braquage (degrés)
var _cmd_queue: Array = []          # ordres reçus [temps, vitesse, direction], appliqués avec un délai
var _clock: float = 0.0
var _next_kick: float = 0.0         # secondes avant le prochain petit choc

# Collision : boîte qui entoure le châssis et les roues (sans la plaquette ni la bille)
var _body_shape := BoxShape3D.new()
var _body_center := Vector3.ZERO    # centre de la boîte, dans le repère de la voiture
var _has_body_box: bool = false
var _own_bodies: Array[RID] = []    # corps de la voiture elle-même (bille, plaquette) : ignorés par le test

func _ready() -> void:
	Realism.setup()
	_next_kick = Realism.rng.randf_range(0.5, 1.5) * Realism.p["kick_interval_s"]
	network = get_node_or_null("/root/Main/NetworkClient")
	_ray = get_node_or_null("RayCast3D")
	if _ray == null:
		push_error("picar_raycast.gd : nœud RayCast3D introuvable, pas de capteur ultrason.")
		set_physics_process(false)
	_wheels = WheelAnimator.new(self)
	_build_collision_box()

func _unhandled_key_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_R:
		Realism.toggle()

# --- Mouvement -----------------------------------------------------------------------------------

func _process(delta: float) -> void:
	# Sans connexion au serveur, la voiture reste strictement immobile
	if network == null or not network.is_connected:
		actual_speed = 0.0
		actual_steer_rad = 0.0
		actual_yaw_rate = 0.0
		return

	var rx_speed: float = float(network.data_received.get("velocity", 0.0))
	var rx_steer: float = float(network.data_received.get("direction", 0.0))
	var command: Vector2 = _apply_latency(rx_speed, rx_steer, delta)
	_update_actual_motion(command.x, command.y, delta)
	actual_yaw_rate = (actual_speed / WHEELBASE) * tan(actual_steer_rad)

	var step := Vector3(-actual_speed * delta, 0, 0)   # l'avant est -X
	var moved: float = actual_speed * delta
	if _would_collide(step, actual_yaw_rate * delta):
		moved = 0.0  # bloquée par un obstacle : la voiture ne bouge pas, les roues non plus
	else:
		translate(step)
		rotate_y(actual_yaw_rate * delta)
		_apply_kick(delta)
	_wheels.animate(moved, actual_steer_rad, delta)

	network.data_to_send["velocity"] = rx_speed

## Retarde les ordres du FSM (latence de la communication). Renvoie (vitesse, direction).
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

## Calcule la vitesse et le braquage RÉELS à partir des ordres (actual_speed, actual_steer_rad)
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

## Petit choc aléatoire (bosse, gravier) : fait pivoter un peu la voiture
func _apply_kick(delta: float) -> void:
	if not Realism.enabled or absf(actual_speed) < 0.05:
		return
	_next_kick -= delta
	if _next_kick > 0.0:
		return
	_next_kick = Realism.rng.randf_range(0.5, 1.5) * Realism.p["kick_interval_s"]
	rotate_y(deg_to_rad(Realism.rng.randf_range(-1.0, 1.0) * Realism.noise("kick_yaw_deg")))

# --- Collision -----------------------------------------------------------------------------------

## Construit la boîte de collision à partir des maillages du châssis et des roues
func _build_collision_box() -> void:
	var to_car: Transform3D = global_transform.affine_inverse()
	var meshes: Array = []
	var model := get_node_or_null("RobotModel")
	if model != null:
		meshes.append_array(model.find_children("*", "MeshInstance3D", true, false))
	for child in get_children():
		if String(child.name).begins_with("Roue"):
			meshes.append_array(child.find_children("*", "MeshInstance3D", true, false))
	if meshes.is_empty():
		return
	var box := AABB()
	var first := true
	for node in meshes:
		var mi := node as MeshInstance3D
		var mesh_box: AABB = (to_car * mi.global_transform) * mi.get_aabb()
		box = mesh_box if first else box.merge(mesh_box)
		first = false
	_body_shape.size = box.size
	_body_center = box.get_center()
	_has_body_box = true
	for body in find_children("*", "CollisionObject3D", true, false):
		_own_bodies.append((body as CollisionObject3D).get_rid())

## true si la voiture entrerait dans un obstacle en faisant ce pas (avance `step`, puis rotation `yaw`)
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

# --- Capteur ultrason ----------------------------------------------------------------------------

func _physics_process(_delta: float) -> void:
	if not (network and network.is_connected):
		return
	var distance: float = NO_ECHO_DISTANCE
	if _ray.is_colliding():
		distance = _ray.get_collision_point().distance_to(global_position)
	# SimBridge attend des centimètres
	network.data_to_send["distance"] = round(Realism.noisy_distance(distance) * 100.0)
