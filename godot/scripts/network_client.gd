class_name NetworkClient
extends Node

## Lien TCP entre Godot et SimBridge (le pont C++), sur le port 9000.
##
## Protocole : des lignes de JSON terminées par "\n", dans les deux sens, à 50 Hz.
##   Godot -> SimBridge : l'état des capteurs (data_to_send).
##   SimBridge -> Godot : les commandes du FSM (data_received). Seules "velocity" et "direction" sont lues.
## Godot envoie aussi deux commandes texte : "trigger_start" (départ) et "emergency_stop" (arrêt d'urgence).

signal connected_to_server
signal disconnected_from_server
signal connection_error(message: String)

const SEND_INTERVAL: float = 0.02  # une trame toutes les 20 ms (50 Hz)

## true quand la connexion TCP est établie
var is_connected: bool = false

## Mis à jour par les capteurs (voir picar_raycast.gd, line_follower_raycast.gd, ball_monitor.gd) puis envoyé tel quel.
## Les cinq clés sont lues par SimBridge (utils/simple_json.hpp).
var data_to_send: Dictionary = {
	"velocity": 0.0,                           # vitesse commandée, renvoyée telle quelle (m/s)
	"distance": 100.0,                         # ultrason (cm)
	"line_follower": [0, 0, 0, 0, 0],          # capteurs de ligne : 0 ou 1 (le FSM lit plutôt line_analog)
	"line_analog": [0.0, 0.0, 0.0, 0.0, 0.0],  # noirceur de chaque capteur, de 0 (blanc) à 1 (noir)
	"ball_dropped": false,                     # la bille est tombée
}

## Dernières commandes reçues du FSM
var data_received: Dictionary = {
	"velocity": 0.0,   # vitesse voulue (m/s)
	"direction": 0.0,  # braquage voulu, de -1 à +1 (fraction du braquage maximal de 48°)
}

var _socket: StreamPeerTCP = StreamPeerTCP.new()
var _rx_buffer: String = ""   # texte reçu, en attente d'un "\n"
var _send_timer: float = 0.0

## Ouvre la connexion (asynchrone) : connected_to_server est émis une fois établie.
func connect_to_server(ip: String, port: int = 9000) -> void:
	_rx_buffer = ""
	is_connected = false
	_socket = StreamPeerTCP.new()
	if _socket.connect_to_host(ip, port) != OK:
		emit_signal("connection_error", "Impossible d'initier TCP vers " + ip + ":" + str(port))
		return
	print("[NetworkClient] Connexion TCP vers ", ip, ":", port)

func disconnect_from_server() -> void:
	_socket.disconnect_from_host()
	is_connected = false
	emit_signal("disconnected_from_server")
	print("[NetworkClient] Déconnecté du serveur.")

## Ordre de départ : SimBridge le transmet au FSM
func send_trigger_start() -> void:
	_send_text_command("trigger_start")

## Arrêt d'urgence : SimBridge le transmet au FSM et aux moteurs
func send_emergency_stop() -> void:
	_send_text_command("emergency_stop")

func _send_text_command(command: String) -> void:
	if not is_connected:
		return
	_socket.put_data((command + "\n").to_utf8_buffer())
	print("[NetworkClient] Commande '", command, "' envoyée.")

func _process(delta: float) -> void:
	_socket.poll()
	var status := _socket.get_status()
	if status == StreamPeerTCP.STATUS_CONNECTED:
		if not is_connected:
			_on_connected()
		_read_commands()
		_send_state_periodically(delta)
	elif status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE:
		if is_connected:
			is_connected = false
			emit_signal("disconnected_from_server")
			print("[NetworkClient] Connexion TCP fermée.")

func _on_connected() -> void:
	is_connected = true
	emit_signal("connected_to_server")
	print("[NetworkClient] Connecté à SimBridge.")
	_send_state()  # première trame tout de suite, sans attendre 20 ms
	_send_timer = 0.0

## Lit ce qui est arrivé et traite chaque ligne complète
func _read_commands() -> void:
	var available := _socket.get_available_bytes()
	if available <= 0:
		return
	_rx_buffer += _socket.get_utf8_string(available)
	var newline := _rx_buffer.find("\n")
	while newline != -1:
		_merge_json_line(_rx_buffer.substr(0, newline).strip_edges())
		_rx_buffer = _rx_buffer.substr(newline + 1)
		newline = _rx_buffer.find("\n")

## Copie les clés d'une ligne JSON dans data_received (les lignes vides ou invalides sont ignorées)
func _merge_json_line(line: String) -> void:
	if line.is_empty():
		return
	var parsed = JSON.parse_string(line)
	if parsed is Dictionary:
		for key in parsed.keys():
			data_received[key] = parsed[key]

func _send_state_periodically(delta: float) -> void:
	_send_timer += delta
	if _send_timer >= SEND_INTERVAL:
		_send_timer = 0.0
		_send_state()

func _send_state() -> void:
	_socket.put_data((JSON.stringify(data_to_send) + "\n").to_utf8_buffer())
