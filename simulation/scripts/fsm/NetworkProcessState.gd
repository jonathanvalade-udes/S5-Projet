class_name NetworkFSMProcessState
extends StateMachineState

var test = 0

func _input(event):
	pass

# Called when the state machine enters this state.
func on_enter() -> void:
	print("Network Process State entered")
	#get_parent().socket.send_text("Hello mom!")


# Called every frame when this state is active.
func on_process(delta: float) -> void:
	get_parent().socket.poll()
	var state = get_parent().socket.get_ready_state()
	if state == WebSocketPeer.STATE_OPEN:

		# Received packets and process them
		while get_parent().socket.get_available_packet_count():
			var raw_packet = get_parent().socket.get_packet().get_string_from_utf8()
			var parsed_data = JSON.parse_string(raw_packet)
			if parsed_data is Dictionary:
				for key in parsed_data.keys():
					get_parent().data_received[key] = parsed_data[key]
			else:
				print("Error while parsing received string or non-dictionary payload")

		# Send current data to send JSON packet at 50Hz (every 20ms) for snappy reactive steering
		if test >= 0.02:
			var json_data = JSON.stringify(get_parent().data_to_send).to_utf8_buffer()
			get_parent().socket.send(json_data)
			test = 0
		else:
			test += delta
	elif state == WebSocketPeer.STATE_CLOSING or state == WebSocketPeer.STATE_CLOSED:
		get_parent().current_state = null


# Called every physics frame when this state is active.
func on_physics_process(delta: float) -> void:
	pass


# Called when there is an input event while this state is active.
func on_input(event: InputEvent) -> void:
	pass


# Called when the state machine exits this state.
func on_exit() -> void:
	print("Network Process State left")
