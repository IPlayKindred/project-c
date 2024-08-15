extends Node3D

@export var curve:Curve

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


var set_rot = true

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	
	if is_instance_valid(Car.car):
		
		var target = Car.car
		
		global_transform.origin = Car.camera.global_transform.origin
		
		
		
		self.rotation_degrees = Vector3(target.rotation_degrees.x, self.rotation_degrees.y, 0)
		
		var yaw_factor = (angle_to_object(self, target))
		
		yaw_factor = clamp(abs(yaw_factor) / 10, 0, 1)
		
		self.rotation_degrees.y = self.rotation_degrees.y - (angle_to_object(self, target) * 10 / 100)
		
		
		if set_rot:
			Car.camera.rotation_degrees.y = self.rotation_degrees.y
			Car.camera.get_child(0).yaw_factor = -yaw_factor * 0.15 * curve.sample(yaw_factor)
	

func angle_to_object(sel, obj):
	var angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(obj.global_transform.basis.z)))
	
	var right_angle = rad_to_deg(acos(sel.global_transform.basis.x.normalized().dot(obj.global_transform.basis.z)))
	var diff
	
	if right_angle == 90:
		diff = 0
	elif right_angle < 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff
