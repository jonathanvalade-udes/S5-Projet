extends RigidBody3D

# Détection de renversement de la bille (chargement de breuvage FastDrink)
# Profondeur réelle de la cuvette = 1.50 mm (0.0015 m)
# Rayon du trou = 20 mm (0.020 m)

var initial_position: Vector3 = Vector3.ZERO
var is_dropped: bool = false
var is_settled: bool = false
var settle_timer: float = 0.5

signal ball_dropped_event(drop_position: Vector3)


func _ready():
	initial_position = global_position
	freeze = false


func _physics_process(delta):
	# Période de stabilisation initiale (permet à la bille de se caler au fond de la cuvette)
	if not is_settled:
		settle_timer -= delta
		if settle_timer <= 0.0:
			is_settled = true
			initial_position = global_position
			print("[FastDrink] Bille stabilisée dans la coupelle à : ", initial_position)
		return

	# Si la bille est déjà déclarée tombée, on continue d'émettre l'état
	if is_dropped:
		_notify_fsm(true)
		return

	# La position relative par rapport au parent
	var local_pos = position
	var delta_xz = Vector2(local_pos.x, local_pos.z).length()
	var delta_y = abs(local_pos.y - 0.138)

	# Seuil de déversement : sortie de cuvette (> 0.025 m) ou chute verticale (> 0.015 m)
	if delta_xz > 0.025 or delta_y > 0.02:
		is_dropped = true
		emit_signal("ball_dropped_event", global_position)
		print("[ALERTE FastDrink] La bille est tombée ! Déversement du breuvage à: ", global_position)
		_notify_fsm(true)
	else:
		_notify_fsm(false)




func _notify_fsm(dropped_state: bool):
	var nfsm = get_node_or_null("/root/Main/NetworkFSM")
	if nfsm and ("data_to_send" in nfsm):
		nfsm.data_to_send["ball_dropped"] = dropped_state

