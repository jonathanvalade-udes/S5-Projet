class_name NetworkClient
extends Node

## Client WebSocket natif et asynchrone pour FastDrink sous Godot 4.
## Échange bidirectionnel cadencé à 50 Hz avec le serveur Python (robot/main.py).

signal connected_to_server
signal disconnected_from_server
signal connection_error(message: String)

var socket: WebSocketPeer = WebSocketPeer.new()
var is_connected: bool = false
var send_timer: float = 0.0
const SEND_INTERVAL: float = 0.02  # 50 Hz (20 ms)

var data_to_send: Dictionary = {
	"velocity": 0.0,
	"line_follower": [0, 0, 0, 0, 0],
	"ball_dropped": false,
	"distance": 100.0
}

var data_received: Dictionary = {
	"velocity": 0.0,
	"direction": 0.0,
	"distance": 100.0,
	"line_follower": [0, 0, 0, 0, 0]
}

func connect_to_server(ip: String, port: int = 8765) -> void:
	socket = WebSocketPeer.new()
	var url = "ws://" + ip + ":" + str(port)
	var err = socket.connect_to_url(url)
	if err != OK:
		emit_signal("connection_error", "Impossible d'initier la connexion vers " + url)
		return
	print("[NetworkClient] Tentative de connexion vers: ", url)

func disconnect_from_server() -> void:
	if is_connected or socket.get_ready_state() == WebSocketPeer.STATE_CONNECTING:
		socket.close(1000, "Déconnexion manuelle utilisateur")
	is_connected = false
	emit_signal("disconnected_from_server")
	print("[NetworkClient] Déconnecté du serveur.")

func _process(delta: float) -> void:
	socket.poll()
	var state = socket.get_ready_state()
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected:
			is_connected = true
			emit_signal("connected_to_server")
			print("[NetworkClient] Connecté avec succès au serveur robot !")
		
		# Réception et traitement des trames JSON entrantes
		while socket.get_available_packet_count() > 0:
			var raw_packet = socket.get_packet().get_string_from_utf8()
			var parsed = JSON.parse_string(raw_packet)
			if parsed is Dictionary:
				for k in parsed.keys():
					data_received[k] = parsed[k]
		
		# Envoi périodique à 50 Hz (20 ms)
		send_timer += delta
		if send_timer >= SEND_INTERVAL:
			send_timer = 0.0
			var payload = JSON.stringify(data_to_send).to_utf8_buffer()
			socket.send(payload)
			
	elif state == WebSocketPeer.STATE_CLOSED or state == WebSocketPeer.STATE_CLOSING:
		if is_connected:
			is_connected = false
			emit_signal("disconnected_from_server")
			print("[NetworkClient] La connexion avec le serveur a été fermée.")
