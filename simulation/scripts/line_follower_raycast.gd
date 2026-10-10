extends Area3D

@export var line_follower_array: PackedByteArray = [0, 0, 0, 0, 0]

var network: NetworkClient = null

func _ready() -> void:
	network = get_node_or_null("/root/Main/NetworkClient")

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

func _on_body_shape_entered(_body_rid, body, _body_shape_index, local_shape_index) -> void:	
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 1

func _on_body_shape_exited(_body_rid, body, _body_shape_index, local_shape_index) -> void:
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 0

func _on_area_exited(_area: Area3D) -> void:
	pass
