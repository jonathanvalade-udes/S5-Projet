extends RigidBody3D

# Détection et dynamique physique de la bille (chargement FastDrink)
# Coupelle conique : Rayon R = 20 mm (0.020 m), profondeur h = 1.50 mm (0.0015 m)
# Pente conique : tan(theta) = h / R = 0.075 -> Accélération critique a_max = g * tan(theta) = 0.735 m/s²
# Bille : diamètre 25 mm (rayon 12.5 mm), masse m = 0.065 kg

const CUP_RADIUS: float = 0.020        # 20 mm
const CUP_DEPTH: float = 0.0015        # 1.5 mm
const SLOPE_TAN: float = 0.075         # h / R
const REST_Y: float = 0.138            # Hauteur de repos au fond de la cuvette
const G: float = 9.81
const MASS: float = 0.065
const DAMPING: float = 6.0             # Coefficient d'amortissement de roulement

var is_dropped: bool = false
var ball_rel_pos: Vector2 = Vector2.ZERO   # Position locale (x, z) dans le plan de la coupelle
var ball_rel_vel: Vector2 = Vector2.ZERO   # Vitesse locale (vx, vz)

var last_speed: float = 0.0
var has_last_speed: bool = false

var fall_vel_y: float = 0.0            # Vitesse verticale de chute hors de la voiture

var nfsm: Node = null
var sphere_mesh: Node3D = null

signal ball_dropped_event(drop_position: Vector3)

func _ready():
	freeze = true  # Déplacement physique calculé dans le référentiel non-inertiel du véhicule
	nfsm = get_node_or_null("/root/Main/NetworkFSM")
	sphere_mesh = get_node_or_null("CSGSphere3D")
	position = Vector3(0.0, REST_Y, 0.0)

func _physics_process(delta: float):
	if nfsm == null:
		nfsm = get_node_or_null("/root/Main/NetworkFSM")
	
	if is_dropped:
		# La bille est tombée : chute libre physique hors du véhicule vers le sol
		fall_vel_y -= G * delta
		position.y += fall_vel_y * delta
		position.x += ball_rel_vel.x * delta
		position.z += ball_rel_vel.y * delta
		_notify_fsm(true)
		return

	# Lire la cinématique du PiCar depuis la télémesure
	var rx_speed: float = 0.0
	var rx_steer: float = 0.0
	if nfsm and ("data_received" in nfsm):
		rx_speed = nfsm.data_received.get("velocity", 0.0)
		rx_steer = nfsm.data_received.get("direction", 0.0)

	# 1. Calcul des accélérations du véhicule (référentiel non-inertiel)
	if not has_last_speed:
		last_speed = rx_speed
		has_last_speed = true

	var dt = max(1e-4, delta)
	var a_long = (rx_speed - last_speed) / dt
	last_speed = rx_speed

	# Accélération centripète : a_centri = v * omega, dirigée vers l'extérieur du virage
	var steer_rad = rx_steer * deg_to_rad(48.0)
	var angular_vel = (rx_speed / 0.14) * tan(steer_rad)
	var a_lat = rx_speed * angular_vel

	# 2. Forces d'inertie fictives dans le repère local du véhicule :
	# Avancer = axe -X local => a_long > 0 pousse vers +X (arrière du véhicule)
	# Virage à droite (steer > 0) => force centrifuge pousse vers +Z (extérieur gauche)
	var f_inertie_x = a_long
	var f_inertie_z = a_lat

	# 3. Force de rappel de gravité vers le centre de la cuvette conique
	var dist_center = ball_rel_pos.length()
	var f_grav_x = 0.0
	var f_grav_z = 0.0
	if dist_center > 0.0001:
		var dir = ball_rel_pos / dist_center
		f_grav_x = -G * SLOPE_TAN * dir.x
		f_grav_z = -G * SLOPE_TAN * dir.y

	# 4. Amortissement visqueux de roulement
	var f_damp_x = -DAMPING * ball_rel_vel.x
	var f_damp_z = -DAMPING * ball_rel_vel.y

	# 5. Équation du mouvement (accélération effective de roulement a = 5/7 * F/m)
	var factor = 5.0 / 7.0
	var acc_x = factor * (f_inertie_x + f_grav_x + f_damp_x)
	var acc_z = factor * (f_inertie_z + f_grav_z + f_damp_z)

	ball_rel_vel.x += acc_x * dt
	ball_rel_vel.y += acc_z * dt
	ball_rel_pos.x += ball_rel_vel.x * dt
	ball_rel_pos.y += ball_rel_vel.y * dt

	# 6. Vérifier la condition de renversement (franchissement du rebord de la cuvette)
	if dist_center > CUP_RADIUS:
		is_dropped = true
		fall_vel_y = 0.0
		emit_signal("ball_dropped_event", global_position)
		print("[ALERTE FastDrink] La bille est tombée ! Déversement du breuvage à: ", global_position)
		_notify_fsm(true)
		return

	# 7. Mise à jour de la position 3D dans la cuvette
	var height_y = REST_Y + (dist_center / CUP_RADIUS) * CUP_DEPTH
	position = Vector3(ball_rel_pos.x, height_y, ball_rel_pos.y)

	# 8. Animation de roulement de la bille sur elle-même
	if sphere_mesh:
		sphere_mesh.rotate_z(-ball_rel_vel.x * dt / 0.0125)
		sphere_mesh.rotate_x(ball_rel_vel.y * dt / 0.0125)

	_notify_fsm(false)

func _notify_fsm(dropped_state: bool):
	if nfsm and ("data_to_send" in nfsm):
		nfsm.data_to_send["ball_dropped"] = dropped_state
