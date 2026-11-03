extends Node3D

var NetworkIPAddrRegex: RegEx = RegEx.new()
@onready var network_client: NetworkClient = $NetworkClient
@onready var btn_connect: Button = $GridContainer/btn_Connect
@onready var lb_status: Label = $GridContainer/lb_ConnectionStatusPackets
@onready var le_ip: LineEdit = $GridContainer/le_IpAdress

func _ready() -> void:
	NetworkIPAddrRegex.compile(r'^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)(\.(?!$)|$)){4}(:\d+)?$')
	
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

func _on_check_box_toggled(toggled_on: bool) -> void:
	if toggled_on:
		le_ip.text = "127.0.0.1"
		_on_connect_pressed()
