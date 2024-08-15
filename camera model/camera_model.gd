extends Node3D

@export var active = false
@export var speed_curve:Curve

var target:Node

@export var main_cam:Node

@export var offset:Vector3
var offset2:Vector3
@export var offsetr:Vector3



@export var lerp_speed = 120.0


@export_range(0, 1, 0.001) var rot_speed := 0.95

var lerp


# Called when the node enters the scene tree for the first time.
func _ready():
	Car.camera = self
	pitch = get_parent().get_node("pitch handler")
	yaw = get_parent().get_node("yaw handler")
	if is_instance_valid(Car.car) : target = Car.car
	

var speed_move_up = 0
var move_side = 0
var angle = 0
var current_angle = 0

var shake_offset = Vector3.ZERO
var shake_interpolation

var pitch
var yaw

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	
	
	if is_instance_valid(Car.car):
		target = Car.car
	
	if active == true:
		
		current_angle = target.steer
		
		if angle > current_angle:
			angle = angle - delta
		elif angle < current_angle:
			angle = angle + delta
		else:
			angle = current_angle
		
		
		
		pitch.set_rot = true
		yaw.set_rot = true
		
		speed_move_up = speed_curve.sample(clamp((target.forward_speed) / 15, 0, 1)) * 0.175
		move_side = angle * -0.4 * speed_curve.sample(clamp((target.forward_speed) / 5, 0, 1))
		
		
		var speed_factor = 100 * (1 - Car.camera.get_child(0).angle_to_fov.sample(Car.camera.get_child(0).angle_ratio))
		
		var target_xform = target.global_transform.translated_local(offset + offset2 + (Vector3.UP * speed_move_up) + (Vector3.RIGHT * move_side) + shake_offset)
		
		lerp = (global_transform.interpolate_with(target_xform, (lerp_speed + speed_factor) * delta).origin)
		
		global_transform.origin = lerp
		
		lerp = (global_transform.interpolate_with(target_xform, lerp_speed * 4 * ((1 / delta) / 100) * delta).origin)
		
		global_transform.origin.y = lerp.y
		
		
		var adjustment = (1 * ((Engine.physics_ticks_per_second) / 260))
		
		var slip_interpolation = clamp((target.forward_speed - 2) / 10, 0, 1) * adjustment
		
		shake_interpolation = speed_curve.sample(clamp((target.forward_speed - 25) / 110, 0, 1)) * 0.55 * ((1 / delta) / 260)
		
		#shake_offset =  Vector3(0, shake_interpolation * randf_range(-1, 1) * 0.5, 0)
	else:
		pitch.set_rot = false
		yaw.set_rot = false
	

func angle_to_object(sel, obj):
	var angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(obj.global_transform.basis.z.normalized())))
	
	var right_angle = rad_to_deg(acos(sel.global_transform.basis.x.normalized().dot(obj.global_transform.basis.z.normalized())))
	var diff
	
	if right_angle == 90:
		diff = 0
	elif right_angle < 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff

func angle_to_object_ver(sel, obj):
	var obj_rotated = obj.global_transform.basis.rotated(obj.global_transform.basis.y, -deg_to_rad((angle_to_object(self, target))))
	
	var angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(obj_rotated.z)))
	
	var right_angle = rad_to_deg(acos(sel.global_transform.basis.y.normalized().dot(obj_rotated.z)))
	var diff
	
	if right_angle == 90:
		diff = 0
	elif right_angle > 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff

func angle_to_vector(sel, vect):
	var angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(vect)))
	
	var right_angle = rad_to_deg(acos(sel.global_transform.basis.x.normalized().dot(vect)))
	var diff
	
	if right_angle == 90:
		diff = 0
	elif right_angle < 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff
