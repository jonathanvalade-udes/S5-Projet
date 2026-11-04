extends Area3D

@export var line_follower_array: PackedByteArray = [0, 0, 0, 0, 0]

# --- Lecture analogique -------------------------------------------------------
# Chaque capteur couvre ~19 mm (pas entre capteurs). On lance plusieurs rayons verticaux répartis
# sur cette largeur : la fraction de rayons qui touche le ruban donne une "noirceur" continue
# [0.0 = blanc, 1.0 = noir], comme un vrai capteur TCRT5000 lu en ADC (au lieu de 0 ou 1).
const SENSOR_PITCH: float = 0.019       # Largeur couverte par un capteur (m)
const SAMPLES_PER_SENSOR: int = 15      # Rayons par capteur (plus = mesure moins "en escalier", donc moins de tremblement)
const RAY_DOWN: float = 0.15            # Longueur du rayon vers le bas (m)
const TAPE_LAYER_MASK: int = 2          # Couche de collision du ruban (voir main.gd)

var network: NetworkClient = null
var _sensor_shapes: Array[CollisionShape3D] = []
var line_analog: Array = [0.0, 0.0, 0.0, 0.0, 0.0]

func _ready() -> void:
	network = get_node_or_null("/root/Main/NetworkClient")
	# Ordre des enfants = indice de forme = [G2, G1, C, D1, D2]
	for child in get_children():
		if child is CollisionShape3D:
			_sensor_shapes.append(child)

func _process(_delta: float) -> void:
	if network == null:
		network = get_node_or_null("/root/Main/NetworkClient")
	if network and network.is_connected:
		network.data_to_send["line_follower"] = [
			int(line_follower_array[0]),
			int(line_follower_array[1]),
			int(line_follower_array[2]),
			int(line_follower_array[3]),
			int(line_follower_array[4])
		]

func _physics_process(_delta: float) -> void:
	_update_analog()
	if network and network.is_connected:
		network.data_to_send["line_analog"] = line_analog.duplicate()

func _update_analog() -> void:
	if _sensor_shapes.size() != 5:
		return
	var space := get_world_3d().direct_space_state
	for i in range(5):
		var shape_node := _sensor_shapes[i]
		var origin := shape_node.global_position
		# Axe latéral du capteur (axe X local du capteur, perpendiculaire à l'avance)
		var lateral := shape_node.global_transform.basis.x.normalized()
		var hits := 0
		for k in range(SAMPLES_PER_SENSOR):
			var offset := ((float(k) + 0.5) / float(SAMPLES_PER_SENSOR) - 0.5) * SENSOR_PITCH
			var from := origin + lateral * offset
			var to := from + Vector3.DOWN * RAY_DOWN
			var query := PhysicsRayQueryParameters3D.create(from, to, TAPE_LAYER_MASK)
			var result := space.intersect_ray(query)
			if not result.is_empty():
				hits += 1
		line_analog[i] = snappedf(float(hits) / float(SAMPLES_PER_SENSOR), 0.001)

func _on_body_shape_entered(_body_rid, body, _body_shape_index, local_shape_index) -> void:
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 1

func _on_body_shape_exited(_body_rid, body, _body_shape_index, local_shape_index) -> void:
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 0
