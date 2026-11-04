extends Node3D

var NetworkIPAddrRegex: RegEx = RegEx.new()
@onready var network_client: NetworkClient = $NetworkClient
@onready var btn_connect: Button = $GridContainer/btn_Connect
@onready var lb_status: Label = $GridContainer/lb_ConnectionStatusPackets
@onready var le_ip: LineEdit = $GridContainer/le_IpAdress
@onready var btn_start_fsm: Button = $GridContainer/btn_StartFSM
@onready var btn_stop_fsm: Button = $GridContainer/btn_StopFSM

func _ready() -> void:
	NetworkIPAddrRegex.compile(r'^((25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)(:\d+)?$')
	
	# Connecter les signaux réseau de NetworkClient
	if network_client:
		network_client.connected_to_server.connect(_on_network_connected)
		network_client.disconnected_from_server.connect(_on_network_disconnected)
		network_client.connection_error.connect(_on_network_error)

	# Configurer le CircuitParcours sur la collision_layer 2 (ruban optique)
	var circuit = get_node_or_null("CircuitParcours")
	if circuit:
		_set_collision_layer_recursive(circuit, 2)

func _set_collision_layer_recursive(node: Node, layer: int) -> void:
	if node is StaticBody3D:
		node.collision_layer = layer
		node.collision_mask = layer
	for child in node.get_children():
		_set_collision_layer_recursive(child, layer)

func _on_network_connected() -> void:
	btn_connect.disabled = false
	btn_connect.text = "Disconnect"
	lb_status.text = "Connected!"

func _on_network_disconnected() -> void:
	btn_connect.disabled = false
	btn_connect.text = "Connect"
	lb_status.text = "Disconnected"

func _on_network_error(msg: String) -> void:
	btn_connect.disabled = false
	btn_connect.text = "Connect"
	lb_status.text = "Error: " + msg

func _on_quit_pressed() -> void:
	if network_client:
		network_client.disconnect_from_server()
	get_tree().quit()

func _on_connect_pressed() -> void:
	if network_client.is_connected:
		network_client.disconnect_from_server()
	else:
		var ip_str = le_ip.text.strip_edges()
		if NetworkIPAddrRegex.search(ip_str):
			btn_connect.disabled = true
			lb_status.text = "Connecting..."
			var target_port: int = 9000
			var target_ip: String = ip_str
			if ip_str.find(":") != -1:
				var parts = ip_str.split(":")
				target_ip = parts[0]
				target_port = int(parts[1])
			network_client.connect_to_server(target_ip, target_port)
		else:
			lb_status.text = "Wrong IP Address!"

func _on_start_fsm_pressed() -> void:
	if network_client and network_client.is_connected:
		network_client.send_trigger_start()
		lb_status.text = "FSM Démarré !"
	else:
		lb_status.text = "Erreur: Non connecté !"

func _on_stop_fsm_pressed() -> void:
	if network_client and network_client.is_connected:
		network_client.send_emergency_stop()
		lb_status.text = "Arrêt d'urgence FSM"
	else:
		lb_status.text = "Erreur: Non connecté !"
