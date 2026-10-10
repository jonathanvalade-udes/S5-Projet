class_name NetworkFSMClosingConnectionState
extends StateMachineState


# Called when the state machine enters this state.
func on_enter() -> void:
	print("Network Closing connection State entered")
	get_parent().socket.close(1000, "Connection was closed by the user")
	get_node("../../AspectRatioContainer/GridContainer/btn_Connect").text = "Connect"
	get_node("../../AspectRatioContainer/GridContainer/lb_ConnectionStatusPackets").text = "Disconnected"
	get_parent().current_state = $"../NetworkWaitingState"


# Called every frame when this state is active.
func on_process(delta: float) -> void:
	pass

# Called every physics frame when this state is active.
func on_physics_process(delta: float) -> void:
	pass

# Called when there is an input event while this state is active.
func on_input(event: InputEvent) -> void:
	pass

# Called when the state machine exits this state.
func on_exit() -> void:
	print("Network Closing connection State left")
