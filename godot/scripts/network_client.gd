class_name NetworkClient
extends Node

## Client Réseau Hybride (TCP Direct & WebSocket) pour FastDrink sous Godot 4.
## - Port 9000 : TCP Direct natif ultra-rapide vers le pont C++ (picar_sim_bridge).
## - Port 8765 : WebSocket hérité pour tests Python.

signal connected_to_server
signal disconnected_from_server
signal connection_error(message: String)

var tcp_socket: StreamPeerTCP = StreamPeerTCP.new()
var ws_socket: WebSocketPeer = WebSocketPeer.new()

var is_tcp_mode: bool = true
var is_connected: bool = false
var send_timer: float = 0.0
const SEND_INTERVAL: float = 0.02  # 50 Hz (20 ms)

var rx_buffer: String = ""

var data_to_send: Dictionary = {
	"velocity": 0.0,
	"line_follower": [0, 0, 0, 0, 0],
	"line_analog": [0.0, 0.0, 0.0, 0.0, 0.0],  # noirceur continue par capteur [0,1] (voir line_follower_raycast.gd)
	"ball_dropped": false,
	"distance": 100.0
}

var data_received: Dictionary = {
	"velocity": 0.0,
	"direction": 0.0,
	"distance": 100.0,
	"line_follower": [0, 0, 0, 0, 0]
}

func connect_to_server(ip: String, port: int = 9000) -> void:
	rx_buffer = ""
	is_connected = false
	
	if port == 8765:
		is_tcp_mode = false
		ws_socket = WebSocketPeer.new()
		var url = "ws://" + ip + ":" + str(port)
		var err = ws_socket.connect_to_url(url)
		if err != OK:
			emit_signal("connection_error", "Impossible d'initier WebSocket vers " + url)
			return
		print("[NetworkClient] Connexion WebSocket vers: ", url)
	else:
		is_tcp_mode = true
		tcp_socket = StreamPeerTCP.new()
		var err = tcp_socket.connect_to_host(ip, port)
		if err != OK:
			emit_signal("connection_error", "Impossible d'initier TCP vers " + ip + ":" + str(port))
			return
		print("[NetworkClient] Connexion TCP directe vers: ", ip, ":", port)

func disconnect_from_server() -> void:
	if is_tcp_mode:
		if tcp_socket:
			tcp_socket.disconnect_from_host()
	else:
		if is_connected or ws_socket.get_ready_state() == WebSocketPeer.STATE_CONNECTING:
			ws_socket.close(1000, "Déconnexion manuelle")
	
	is_connected = false
	emit_signal("disconnected_from_server")
	print("[NetworkClient] Déconnecté du serveur.")

## Envoie l'ordre de top départ au FSM via le pont C++ (/tmp/peripherals.sock)
func send_trigger_start() -> void:
	if is_tcp_mode and tcp_socket and is_connected:
		var cmd = "trigger_start\n"
		tcp_socket.put_data(cmd.to_utf8_buffer())
		print("[NetworkClient TCP] Commande 'trigger_start' envoyée au pont C++.")
	elif not is_tcp_mode and ws_socket and is_connected:
		var payload = JSON.stringify({"command": "trigger_start"}).to_utf8_buffer()
		ws_socket.send(payload)
		print("[NetworkClient WS] Commande 'trigger_start' envoyée.")

## Envoie un arrêt d'urgence au robot / simulateur
func send_emergency_stop() -> void:
	if is_tcp_mode and tcp_socket and is_connected:
		var cmd = "emergency_stop\n"
		tcp_socket.put_data(cmd.to_utf8_buffer())
		print("[NetworkClient TCP] Commande 'emergency_stop' envoyée.")
	elif not is_tcp_mode and ws_socket and is_connected:
		var payload = JSON.stringify({"command": "emergency_stop"}).to_utf8_buffer()
		ws_socket.send(payload)
		print("[NetworkClient WS] Commande 'emergency_stop' envoyée.")

func _process(delta: float) -> void:
	if is_tcp_mode:
		_process_tcp(delta)
	else:
		_process_websocket(delta)

func _process_tcp(delta: float) -> void:
	tcp_socket.poll()
	var status = tcp_socket.get_status()
	
	if status == StreamPeerTCP.STATUS_CONNECTED:
		if not is_connected:
			is_connected = true
			emit_signal("connected_to_server")
			print("[NetworkClient TCP] Connecté avec succès au pont C++ !")
			# Envoyer immédiatement une première trame dès la connexion
			var init_line = JSON.stringify(data_to_send) + "\n"
			tcp_socket.put_data(init_line.to_utf8_buffer())
			send_timer = 0.0
		
		# Lecture du flux TCP brut et découpage ligne par ligne (\n)
		var available = tcp_socket.get_available_bytes()
		if available > 0:
			var chunk = tcp_socket.get_utf8_string(available)
			rx_buffer += chunk
			
			while rx_buffer.find("\n") != -1:
				var nl_idx = rx_buffer.find("\n")
				var line = rx_buffer.substr(0, nl_idx).strip_edges()
				rx_buffer = rx_buffer.substr(nl_idx + 1)
				
				if not line.is_empty():
					var parsed = JSON.parse_string(line)
					if parsed is Dictionary:
						for k in parsed.keys():
							data_received[k] = parsed[k]
		
		# Envoi périodique à 50 Hz
		send_timer += delta
		if send_timer >= SEND_INTERVAL:
			send_timer = 0.0
			var line_to_send = JSON.stringify(data_to_send) + "\n"
			tcp_socket.put_data(line_to_send.to_utf8_buffer())
			
	elif status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE:
		if is_connected:
			is_connected = false
			emit_signal("disconnected_from_server")
			print("[NetworkClient TCP] Connexion TCP fermée.")

func _process_websocket(delta: float) -> void:
	ws_socket.poll()
	var state = ws_socket.get_ready_state()
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected:
			is_connected = true
			emit_signal("connected_to_server")
			print("[NetworkClient WS] Connecté avec succès au serveur WebSocket !")
		
		while ws_socket.get_available_packet_count() > 0:
			var raw_packet = ws_socket.get_packet().get_string_from_utf8()
			var parsed = JSON.parse_string(raw_packet)
			if parsed is Dictionary:
				for k in parsed.keys():
					data_received[k] = parsed[k]
		
		send_timer += delta
		if send_timer >= SEND_INTERVAL:
			send_timer = 0.0
			var payload = JSON.stringify(data_to_send).to_utf8_buffer()
			ws_socket.send(payload)
			
	elif state == WebSocketPeer.STATE_CLOSED or state == WebSocketPeer.STATE_CLOSING:
		if is_connected:
			is_connected = false
			emit_signal("disconnected_from_server")
			print("[NetworkClient WS] Connexion WebSocket fermée.")
