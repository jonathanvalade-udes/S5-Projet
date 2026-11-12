extends Node3D

## Scène principale : prépare le monde au démarrage et relie les boutons au réseau.
##
## Au démarrage :
##   - donne au ruban du circuit la couche de collision 2 (les capteurs de ligne ne voient que cette couche) ;
##   - donne une collision aux cubes "obstacle*" (couche 1), pour que la voiture soit bloquée et que l'ultrason les voie ;
##   - lit les options de la ligne de commande (voir plus bas).
##
## Options de la ligne de commande, après "--" :
##   --time-scale <N>   : accélère la simulation N fois (tests rapides) ;
##   --optuna           : mode d'évaluation automatique, voir optuna_runner.gd ;
##   --timeout <s>      : limite de temps du mode Optuna (35 s par défaut).

const OptunaRunner = preload("res://scripts/optuna_runner.gd")

const TAPE_LAYER: int = 2       # couche de collision du ruban
const DEFAULT_PORT: int = 9000  # port de SimBridge
const DEFAULT_TIMEOUT: float = 35.0

@onready var network_client: NetworkClient = $NetworkClient
@onready var btn_connect: Button = $GridContainer/btn_Connect
@onready var lb_status: Label = $GridContainer/lb_ConnectionStatusPackets
@onready var le_ip: LineEdit = $GridContainer/le_IpAdress

var _ip_regex := RegEx.new()
var _optuna: OptunaRunner = null

func _ready() -> void:
	# Adresse IPv4 avec un port facultatif, par exemple 127.0.0.1:9000
	_ip_regex.compile(r'^((25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)\.){3}(25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)(:\d+)?$')

	network_client.connected_to_server.connect(_on_network_connected)
	network_client.disconnected_from_server.connect(_on_network_disconnected)
	network_client.connection_error.connect(_on_network_error)

	_set_collision_layer_recursive($CircuitParcours, TAPE_LAYER)
	for node in get_children():
		if String(node.name).begins_with("obstacle"):
			_add_obstacle_collision(node)

	var timeout := DEFAULT_TIMEOUT
	var optuna_mode := false
	var args: PackedStringArray = OS.get_cmdline_user_args()
	if args.is_empty():
		args = OS.get_cmdline_args()
	for i in range(args.size()):
		if args[i] == "--optuna":
			optuna_mode = true
		elif args[i] == "--time-scale" and i + 1 < args.size():
			Engine.time_scale = float(args[i + 1])
		elif args[i] == "--timeout" and i + 1 < args.size():
			timeout = float(args[i + 1])
	if optuna_mode:
		_optuna = OptunaRunner.new(self, network_client, get_node_or_null("PiCar"), get_node_or_null("PiCar/Bille"), timeout)

func _physics_process(delta: float) -> void:
	if _optuna:
		_optuna.step(delta)

# --- Préparation du monde ------------------------------------------------------------------------

## Met la couche de collision des corps statiques d'un nœud et de tous ses descendants
func _set_collision_layer_recursive(node: Node, layer: int) -> void:
	if node is StaticBody3D:
		node.collision_layer = layer
		node.collision_mask = layer
	for child in node.get_children():
		_set_collision_layer_recursive(child, layer)

## Les cubes importés (obstacle.blend) n'ont pas de collision : on en crée une (enveloppe convexe, couche 1)
func _add_obstacle_collision(node: Node) -> void:
	for mesh in node.find_children("*", "MeshInstance3D", true, false):
		var mi := mesh as MeshInstance3D
		var has_body := false
		for child in mi.get_children():
			if child is StaticBody3D:
				has_body = true
		if not has_body:
			mi.create_convex_collision()

# --- Réseau : affichage de l'état de la connexion ------------------------------------------------

func _on_network_connected() -> void:
	_show_status("Connected!", "Disconnect")

func _on_network_disconnected() -> void:
	_show_status("Disconnected")

func _on_network_error(msg: String) -> void:
	_show_status("Error: " + msg)

## Affiche le message d'état et le texte du bouton de connexion (qui redevient cliquable)
func _show_status(message: String, button_text: String = "Connect") -> void:
	btn_connect.disabled = false
	btn_connect.text = button_text
	lb_status.text = message

# --- Boutons -------------------------------------------------------------------------------------

func _on_connect_pressed() -> void:
	if network_client.is_connected:
		network_client.disconnect_from_server()
		return
	var address := le_ip.text.strip_edges()
	if not _ip_regex.search(address):
		lb_status.text = "Wrong IP Address!"
		return
	var ip := address
	var port := DEFAULT_PORT
	if address.find(":") != -1:
		var parts := address.split(":")
		ip = parts[0]
		port = int(parts[1])
	btn_connect.disabled = true
	lb_status.text = "Connecting..."
	network_client.connect_to_server(ip, port)

func _on_start_fsm_pressed() -> void:
	if network_client.is_connected:
		network_client.send_trigger_start()
		lb_status.text = "FSM Démarré !"
	else:
		lb_status.text = "Erreur: Non connecté !"

func _on_stop_fsm_pressed() -> void:
	if network_client.is_connected:
		network_client.send_emergency_stop()
		lb_status.text = "Arrêt d'urgence FSM"
	else:
		lb_status.text = "Erreur: Non connecté !"

func _on_quit_pressed() -> void:
	network_client.disconnect_from_server()
	get_tree().quit()
