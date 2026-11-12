extends RefCounted

## Animation visuelle des roues du PiCar : elles roulent, et les deux roues avant braquent.
## N'a aucun effet sur le mouvement de la voiture (voir picar_raycast.gd) : c'est uniquement de l'affichage.
##
## Les pièces animées sont :
##   - les 4 nœuds "Roue*" (enfants du PiCar) : elles roulent autour de leur axe X local ;
##   - les petits axes de roue du modèle robot.blend : ils ne roulent pas, mais ceux de l'avant suivent le braquage.
## Tout est calculé dans le repère de la voiture : l'avant est -X, le haut est +Y, l'axe des roues est Z.

const ROBOT_MODEL_NODE: String = "RobotModel"
const AXLE_MIN_SIZE: float = 0.5   # un petit axe de roue mesure entre 0,5 et 0,8 unités de modèle
const AXLE_MAX_SIZE: float = 0.8
const STEER_SMOOTHING: float = 15.0  # vitesse du lissage de l'angle de braquage affiché (1/s)

class Part extends RefCounted:
	var node: Node3D
	var rest: Transform3D   # pose de repos, dans le repère de la voiture
	var pivot: Vector3      # point autour duquel la pièce braque, dans le repère de la voiture
	var spin: float         # +1 ou -1 : sens de roulement ; 0 : la pièce ne roule pas
	var front: bool = false # true : la pièce suit le braquage

var _car: Node3D
var _parts: Array[Part] = []
var _wheel_radius: float = 0.0   # rayon d'une roue (m), mesuré sur le modèle
var _wheel_angle: float = 0.0    # angle de roulement actuel (rad)
var _steer_visual: float = 0.0   # angle de braquage affiché (rad), lissé

func _init(car: Node3D) -> void:
	_car = car
	var wheel_count := _find_wheels()
	_find_axles(wheel_count)
	_mark_front_parts(wheel_count)
	print("[PiCar] pièces animées : ", _parts.size(), " dont ", wheel_count, " roues (rayon ", _wheel_radius, " m)")

## distance : mètres parcourus (positif = avant) ; steer_rad : braquage demandé (rad)
func animate(distance: float, steer_rad: float, delta: float) -> void:
	if _parts.is_empty():
		return
	if _wheel_radius > 0.0:
		_wheel_angle = fposmod(_wheel_angle + distance / _wheel_radius, TAU)
	_steer_visual = lerpf(_steer_visual, steer_rad, clampf(delta * STEER_SMOOTHING, 0.0, 1.0))
	var steer_basis := Basis(Vector3.UP, _steer_visual)
	var car_transform: Transform3D = _car.global_transform
	for part in _parts:
		var local := part.rest
		if part.spin != 0.0:
			local = local * Transform3D(Basis(Vector3.RIGHT, _wheel_angle * part.spin), Vector3.ZERO)
		if part.front:
			local = Transform3D(steer_basis, part.pivot - steer_basis * part.pivot) * local
		part.node.global_transform = car_transform * local

## Repère les roues. Renvoie leur nombre (elles sont les premières de la liste).
func _find_wheels() -> int:
	var to_car: Transform3D = _car.global_transform.affine_inverse()
	var count := 0
	for child in _car.get_children():
		if not (child is Node3D) or not String(child.name).begins_with("Roue"):
			continue
		var rest: Transform3D = to_car * (child as Node3D).global_transform
		# Le haut de la roue doit aller vers l'avant (-X) : on tourne dans le sens de l'axe de la roue
		var spin: float = 1.0 if rest.basis.x.z >= 0.0 else -1.0
		for mesh in child.find_children("*", "MeshInstance3D", true, false):
			var mi := mesh as MeshInstance3D
			_wheel_radius = ((to_car * mi.global_transform) * mi.get_aabb()).size.y / 2.0
			break
		_add_part(child as Node3D, rest, rest.origin, spin)
		count += 1
	return count

## Repère les petits axes de roue du modèle : chacun braque autour de la roue la plus proche
func _find_axles(wheel_count: int) -> void:
	var model := _car.get_node_or_null(ROBOT_MODEL_NODE) as Node3D
	if model == null:
		return
	var to_car: Transform3D = _car.global_transform.affine_inverse()
	var to_model: Transform3D = model.global_transform.affine_inverse()
	for node in model.find_children("*", "MeshInstance3D", true, false):
		var mi := node as MeshInstance3D
		var box: AABB = (to_model * mi.global_transform) * mi.get_aabb()
		var largest: float = maxf(box.size.x, maxf(box.size.y, box.size.z))
		if largest <= AXLE_MIN_SIZE or largest > AXLE_MAX_SIZE:
			continue
		var rest: Transform3D = to_car * mi.global_transform
		var center: Vector3 = rest * mi.get_aabb().get_center()
		_add_part(mi, rest, _nearest_wheel_pivot(center, wheel_count), 0.0)

func _nearest_wheel_pivot(center: Vector3, wheel_count: int) -> Vector3:
	var pivot := center
	var best_distance := INF
	for i in range(wheel_count):
		var d: float = absf(_parts[i].pivot.x - center.x) + absf(_parts[i].pivot.z - center.z)
		if d < best_distance:
			best_distance = d
			pivot = _parts[i].pivot
	return pivot

## L'avant = les pièces dont le pivot est plus en avant (-X) que le milieu des roues
func _mark_front_parts(wheel_count: int) -> void:
	var x_mid := 0.0
	for i in range(wheel_count):
		x_mid += _parts[i].pivot.x
	x_mid /= maxf(wheel_count, 1)
	for part in _parts:
		part.front = part.pivot.x < x_mid

func _add_part(node: Node3D, rest: Transform3D, pivot: Vector3, spin: float) -> void:
	var part := Part.new()
	part.node = node
	part.rest = rest
	part.pivot = pivot
	part.spin = spin
	_parts.append(part)
