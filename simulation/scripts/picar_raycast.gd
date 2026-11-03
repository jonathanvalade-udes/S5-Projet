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

func _ready() -> void:
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

func _process(delta: float) -> void:
	if network == null:
		network = get_node_or_null("/root/Main/NetworkClient")

	# Si pas connecté au serveur WebSocket Python, la voiture RESTE STRICTEMENT IMMOBILE
	if network == null or not network.is_connected:
		speed = 0.0
		steer_dir = 0.0
		return

	# Mode connecté : cinématique pilotée exclusivement par Python
	var rx_speed: float = float(network.data_received.get("velocity", 0.0))
	var rx_steer: float = float(network.data_received.get("direction", 0.0))
	
	# Cinématique métrique d'Ackermann
	var steer_rad: float = rx_steer * deg_to_rad(MAX_STEER_DEG)
	var angular_velocity: float = (rx_speed / 0.14) * tan(steer_rad)
	
	translate(Vector3(-rx_speed * delta, 0, 0))
	rotate_y(angular_velocity * delta)
	# Roues : roulement + braquage des roues avant
	_animate_wheels(rx_speed * delta, steer_rad, delta)

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
		network.data_to_send["distance"] = round(distance * 100.0)
