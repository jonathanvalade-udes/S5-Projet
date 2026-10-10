extends Area3D


@export var line_follower_array: PackedByteArray = [0,0,0,0,0]

var nfsm: Node = null

# Called when the node enters the scene tree for the first time.
func _ready():
	nfsm = get_node_or_null("/root/Main/NetworkFSM")

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	if nfsm and ("data_to_send" in nfsm):
		nfsm.data_to_send["line_follower"] = [
			int(line_follower_array[0]),
			int(line_follower_array[1]),
			int(line_follower_array[2]),
			int(line_follower_array[3]),
			int(line_follower_array[4])
		]

func _on_body_shape_entered(_body_rid, body, _body_shape_index, local_shape_index):	
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 1


func _on_body_shape_exited(_body_rid, body, _body_shape_index, local_shape_index):
	if body.collision_layer & 2:
		line_follower_array[local_shape_index] = 0


func _on_area_exited(_area: Area3D) -> void:
	pass # Replace with function body.
