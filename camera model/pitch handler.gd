extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


var set_rot = true

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	if is_instance_valid(Car.car):
		var target = Car.car
		
		var factor = Car.camera.get_child(0).angle_to_fov.sample(Car.camera.get_child(0).angle_ratio)
		
		global_transform.origin = Car.camera.global_transform.origin
		var rotation_speed = Car.camera.speed_curve.sample(clamp((Car.car.forward_speed - 60) / 15, 0, 1)) * -1 * factor
		
		self.rotation_degrees = Vector3(self.rotation_degrees.x + deg_to_rad(rotation_speed + (2.5 * factor)), target.rotation_degrees.y, 0)
		
		self.rotation_degrees.x = self.rotation_degrees.x + ((angle_to_object_ver(self, target)) / 50)
		
		if set_rot:
			Car.camera.rotation_degrees.x = (self.rotation_degrees.x)

func angle_to_object_ver(sel, obj):
	var obj_rotated = obj.global_transform.basis
	var angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(obj_rotated.z)))
	
	var right_angle = rad_to_deg(acos(sel.global_transform.basis.y.normalized().dot(obj_rotated.z)))
	var diff
	
	if right_angle == 90:
		diff = 0
	elif right_angle < 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff
