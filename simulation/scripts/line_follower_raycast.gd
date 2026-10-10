extends Area3D


@export var line_follower_array: PackedByteArray = [0,0,0,0,0]

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass

func _on_body_shape_entered(body_rid, body, body_shape_index, local_shape_index):	
	if body.collision_layer == 2:
		line_follower_array[local_shape_index] = 1
	print(line_follower_array)


func _on_body_shape_exited(body_rid, body, body_shape_index, local_shape_index):
	if body.collision_layer == 2:
		line_follower_array[local_shape_index] = 0


func _on_area_exited(area: Area3D) -> void:
	pass # Replace with function body.
