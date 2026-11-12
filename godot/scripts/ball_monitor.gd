extends RigidBody3D

## La bille posée dans la coupelle conique du PiCar (chargement FastDrink).
##
## Le but du projet : transporter la bille sans qu'elle sorte de la coupelle. Elle sort quand l'accélération de la
## voiture dépasse a_max = g * tan(theta) = 9,81 * 0,075 = 0,735 m/s² (pente tan(theta) = profondeur / rayon).
##
## Le nœud est un RigidBody3D GELÉ : le moteur physique de Godot ne le déplace pas. Ce script calcule lui-même le
## mouvement de la bille dans le repère de la voiture, qui n'est pas un repère inertiel. À chaque pas de physique :
##   - la voiture accélère, donc la bille subit une force d'inertie opposée (en avant/arrière, et en virage) ;
##   - la gravité la rappelle vers le centre de la coupelle (pente constante d'un cône) ;
##   - un amortissement de roulement la ralentit ;
##   - une bille qui roule sans glisser n'accélère qu'aux 5/7 de ce que donnerait la force seule.
## Elle tombe dès qu'elle s'éloigne de plus de 20 mm du centre. main.gd lit is_dropped et max_accel_observed (mode Optuna).

const PiCar = preload("res://scripts/picar_raycast.gd")

const CUP_RADIUS: float = 0.020        # rayon de la coupelle (m)
const CUP_DEPTH: float = 0.0015        # profondeur de la coupelle (m)
const SLOPE_TAN: float = CUP_DEPTH / CUP_RADIUS
const BALL_RADIUS: float = 0.0125      # rayon de la bille (m) : sert à l'animation de roulement
const G: float = 9.81
const DAMPING: float = 6.0             # amortissement de roulement (1/s). Hypothèse : rien n'a été mesuré sur la vraie bille.
const ROLLING_FACTOR: float = 5.0 / 7.0

var is_dropped: bool = false
var max_accel_observed: float = 0.0    # accélération totale maximale vue par la bille (m/s²)

var ball_rel_pos: Vector2 = Vector2.ZERO   # position (x, z) par rapport au centre de la coupelle (m)
var ball_rel_vel: Vector2 = Vector2.ZERO   # vitesse (x, z) dans le repère de la voiture (m/s)

var network: NetworkClient = null

var _car: PiCar = null
var _sphere_mesh: Node3D = null            # l'affichage de la bille
var _rest_pos: Vector3 = Vector3.ZERO      # position de la bille au centre de la coupelle (repère de la voiture)
var _last_speed: float = 0.0
var _has_last_speed: bool = false
var _fall_speed: float = 0.0               # vitesse verticale de chute, une fois la bille sortie (m/s)

func _ready() -> void:
	freeze = true
	network = get_node_or_null("/root/Main/NetworkClient")
	_car = get_parent() as PiCar
	if _car == null:
		push_error("ball_monitor.gd : la bille doit être un enfant direct du PiCar.")
		set_physics_process(false)
	_sphere_mesh = get_node_or_null("CSGSphere3D")
	# Le centre de la coupelle est la position placée dans l'éditeur : ne pas la coder en dur,
	# elle doit suivre la plaquette telle qu'elle est dans la scène.
	_rest_pos = position

func _physics_process(delta: float) -> void:
	if is_dropped:
		_fall(delta)
		return

	# La bille subit le mouvement RÉEL de la voiture (avec les imperfections du mode réaliste)
	var speed := 0.0
	var yaw_rate := 0.0
	if network and network.is_connected:
		speed = _car.actual_speed
		yaw_rate = _car.actual_yaw_rate
	if not _has_last_speed:
		_last_speed = speed
		_has_last_speed = true

	var dt: float = max(1e-4, delta)
	var a_long: float = (speed - _last_speed) / dt  # accélération en avant ; > 0 pousse la bille vers l'arrière (+X)
	_last_speed = speed
	var a_lat: float = speed * yaw_rate             # accélération centripète ; la bille est poussée vers l'extérieur du virage (+Z)

	var a_total := sqrt(a_long * a_long + a_lat * a_lat)
	max_accel_observed = max(max_accel_observed, a_total)

	# La position testée est celle du pas précédent : la bille tombe au pas suivant son franchissement du rebord
	var dist_center := ball_rel_pos.length()
	_integrate(a_long, a_lat, dist_center, dt)
	if dist_center > CUP_RADIUS:
		_drop()
		return
	_place_in_cup(dist_center)
	_roll(dt)
	_notify_network(false)

## Fait avancer la vitesse et la position de la bille d'un pas
func _integrate(a_long: float, a_lat: float, dist_center: float, dt: float) -> void:
	# Rappel de la gravité vers le centre (rien si la bille est déjà au centre, à 0,1 mm près)
	var pull_x := 0.0
	var pull_z := 0.0
	if dist_center > 0.0001:
		var toward_edge := ball_rel_pos / dist_center  # direction unitaire, du centre vers la bille
		pull_x = -G * SLOPE_TAN * toward_edge.x
		pull_z = -G * SLOPE_TAN * toward_edge.y
	ball_rel_vel.x += _rolling_acceleration(a_long, pull_x, ball_rel_vel.x) * dt
	ball_rel_vel.y += _rolling_acceleration(a_lat, pull_z, ball_rel_vel.y) * dt
	ball_rel_pos.x += ball_rel_vel.x * dt
	ball_rel_pos.y += ball_rel_vel.y * dt

## Accélération de roulement sur un axe : inertie + rappel de gravité + amortissement, réduits aux 5/7
func _rolling_acceleration(inertia: float, gravity_pull: float, velocity: float) -> float:
	return ROLLING_FACTOR * (inertia + gravity_pull - DAMPING * velocity)

## La bille monte sur la pente de la coupelle quand elle s'éloigne du centre
func _place_in_cup(dist_center: float) -> void:
	var height := (dist_center / CUP_RADIUS) * CUP_DEPTH
	position = _rest_pos + Vector3(ball_rel_pos.x, height, ball_rel_pos.y)

## Fait tourner l'affichage de la bille dans le sens de son déplacement
func _roll(dt: float) -> void:
	if _sphere_mesh:
		_sphere_mesh.rotate_z(-ball_rel_vel.x * dt / BALL_RADIUS)
		_sphere_mesh.rotate_x(ball_rel_vel.y * dt / BALL_RADIUS)

func _drop() -> void:
	is_dropped = true
	_fall_speed = 0.0
	print("[ALERTE FastDrink] La bille est tombée ! Déversement du breuvage à: ", global_position)
	_notify_network(true)

## Chute libre hors de la voiture, avec la vitesse horizontale qu'avait la bille
func _fall(delta: float) -> void:
	_fall_speed -= G * delta
	position.y += _fall_speed * delta
	position.x += ball_rel_vel.x * delta
	position.z += ball_rel_vel.y * delta
	_notify_network(true)

func _notify_network(dropped: bool) -> void:
	if network and network.is_connected:
		network.data_to_send["ball_dropped"] = dropped
