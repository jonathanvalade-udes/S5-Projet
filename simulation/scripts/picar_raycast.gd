extends Node3D

# Cinématique métrique unifiée S10 pour PiCar-S (Godot 4)
# Empattement L = 0.14 m, angle de braquage max = 48°

var ray: RayCast3D = null
var distance: float = 100.0
var speed: float = 0.0
var steer_dir: float = 0.0

var lfs: Node = null
var network: NetworkClient = null

func _ready() -> void:
	ray = get_node_or_null("RayCast3D")
	var lfs_node = get_node_or_null("line_follower_sensor")
	if lfs_node and lfs_node.get_child_count() > 0:
		lfs = lfs_node.get_child(0)
	network = get_node_or_null("/root/Main/NetworkClient")

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
	var steer_rad: float = rx_steer * deg_to_rad(48.0)
	var angular_velocity: float = (rx_speed / 0.14) * tan(steer_rad)
	
	translate(Vector3(-rx_speed * delta, 0, 0))
	rotate_y(angular_velocity * delta)
	
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
