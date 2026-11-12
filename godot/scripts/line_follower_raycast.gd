extends Node3D

## Les 5 capteurs de ligne du PiCar (sur le vrai robot : des cellules TCRT5000 lues par un convertisseur analogique).
##
## Ordre des capteurs : l'indice 0 est le repère à x = -38 mm, l'indice 4 celui à x = +38 mm (voir line_follower_sensor.tscn).
## SimBridge inverse cet ordre pour le FSM, qui suit la convention du robot réel.
##
## Chaque capteur couvre une bande de 19 mm sous la voiture. On y lance plusieurs rayons verticaux vers le sol :
## la fraction de rayons qui touche le ruban donne la "noirceur" du capteur, de 0 (blanc) à 1 (noir).
## Les deux valeurs envoyées à SimBridge :
##   "line_analog"   : les 5 noirceurs. C'est ce que lit le FSM.
##   "line_follower" : les 5 bits (noirceur > 0,5), gardés pour compatibilité avec le protocole.

const SENSOR_COUNT: int = 5
const SENSOR_PITCH: float = 0.019       # largeur couverte par un capteur (m), égale à l'écart entre deux capteurs
const SAMPLES_PER_SENSOR: int = 15      # rayons par capteur : plus il y en a, plus la mesure est fine
const RAY_DOWN: float = 0.15            # longueur de chaque rayon vers le bas (m)
const TAPE_LAYER_MASK: int = 2          # couche de collision du ruban (réglée dans main.gd)
const DARK_THRESHOLD: float = 0.5       # au-dessus, le capteur voit le ruban (bit à 1)

var network: NetworkClient = null
var line_analog: Array = [0.0, 0.0, 0.0, 0.0, 0.0]
var _sensors: Array[Node3D] = []        # un repère (Marker3D) par capteur : sa position et son axe latéral

func _ready() -> void:
	network = get_node_or_null("/root/Main/NetworkClient")
	for child in get_children():
		if child is Marker3D:
			_sensors.append(child)
	if _sensors.size() != SENSOR_COUNT:
		push_error("line_follower_raycast.gd : %d repères de capteur trouvés, %d attendus." % [_sensors.size(), SENSOR_COUNT])
		set_physics_process(false)

func _physics_process(_delta: float) -> void:
	_update_analog()
	if network and network.is_connected:
		var bits: Array = []
		for value in line_analog:
			bits.append(Realism.noisy_line_bit(1 if value > DARK_THRESHOLD else 0))
		network.data_to_send["line_follower"] = bits
		network.data_to_send["line_analog"] = line_analog.duplicate()

## Mesure la noirceur de chaque capteur avec SAMPLES_PER_SENSOR rayons répartis sur sa largeur
func _update_analog() -> void:
	var space := get_world_3d().direct_space_state
	for i in range(SENSOR_COUNT):
		var origin := _sensors[i].global_position
		var lateral := _sensors[i].global_transform.basis.x.normalized()  # perpendiculaire à l'avance
		var hits := 0
		for k in range(SAMPLES_PER_SENSOR):
			var offset := ((float(k) + 0.5) / float(SAMPLES_PER_SENSOR) - 0.5) * SENSOR_PITCH
			var from := origin + lateral * offset
			var query := PhysicsRayQueryParameters3D.create(from, from + Vector3.DOWN * RAY_DOWN, TAPE_LAYER_MASK)
			if not space.intersect_ray(query).is_empty():
				hits += 1
		line_analog[i] = snappedf(float(hits) / float(SAMPLES_PER_SENSOR), 0.001)
