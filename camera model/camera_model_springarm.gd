extends SpringArm3D

@export var speed_curve:Curve
@onready var angle_to_fov:Curve = Curve.new()
@onready var angle_to_offset:Curve = Curve.new()

@export var offester = 3.0
var offset

var target_position:Vector3 

var target_rot:float

var diff:float

var diff_rot:float

@export var margin_pos:float

@export var margin_rot:float

@export var camera:Node


@export var camera_speed:float

@export var camera_rot_speed:float

var steer = 0
var cam_angle = 0

var self_prev_position:Vector3
var self_speed:Vector3
var self_prev_speed:Vector3
var self_acc:Vector3
# Called when the node enters the scene tree for the first time.
func _ready():
	self.rotation_degrees.y = 180
	
	offset = get_parent().offset
	
	set_angle_to_fov_curve()
	set_angle_to_offset_curve()

var t = 1
var a = 0
var a_prev = 0
var ts = 0
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	
	if get_parent().active == true:
		main(delta)

var selection = 0

var angle_ratio = 0

var angle = [0, 45, 90, 135, 180, 225, 270, 315, 360]

var lerp_diff:float = 0

var lerp_over:float = 0

var ratio:float = 0

var speed_interpolation

var yaw_factor = 0

func main(delta):
	if Input.is_action_pressed("camera_right") && Input.is_action_pressed("camera_back"):
		steer = 1
		
		cam_angle = 315
		#print("yes")
	elif Input.is_action_pressed("camera_left") && Input.is_action_pressed("camera_back"):
		steer = -1
		
		cam_angle = 45
		#print("yes")
	elif Input.is_action_pressed("camera_back"):
		
		
		
		cam_angle = 359
		
	elif Input.is_action_pressed("camera_left"):
		steer = -1
		
		cam_angle = 90
	elif Input.is_action_pressed("camera_right"):
		steer = 1
		
		cam_angle = 270
		
	else:
		steer = 0
		
		cam_angle = 180
	
	ts = self.rotation_degrees.y
	
	a = cam_angle - ts
	
	
	
	if (cam_angle - ts) > 179:
		a = -1
	elif (cam_angle - ts) < 0 && (cam_angle - ts) > -179:
		a = -1
	elif (cam_angle - ts) < -179:
		a = 1
	else:
		a = 1
	
	
	
	if abs(cam_angle - ts) < 2 && cam_angle > 350:
		a = 0
		self.rotation_degrees.y = cam_angle
	elif abs(cam_angle - ts) < 2 && cam_angle < 10:
		a = 0
		self.rotation_degrees.y = cam_angle
	
	
	if ts > (359 - 2)  && a > 0:
		self.rotation_degrees.y = 0
	elif ts < (0 + 2) && a < 0:
		self.rotation_degrees.y = 359
	
	camera_offset()
	
	angle_ratio = (abs(self.rotation_degrees.y - 180) / 180)
	
	
	t = clamp(t, 0, 359)
	
	if (Car.car) != null : $Camera3D.fov = camera_fov(Car.car.forward_speed * 2.2, angle_ratio)
	
	var adjusted_length = 5 - (angle_ratio)
	
	self.spring_length = adjusted_length + yaw_factor
	
	self.rotation_degrees.y += (delta * camera_rot_speed * a) * (speed_curve.sample(abs(cam_angle - ts) / 360) * 2)
	
	if ts > 178 && ts < 182 && cam_angle == 359:
		self.rotation_degrees.y = cam_angle
	elif ts > 315 && ts < 360 && cam_angle == 180:
		self.rotation_degrees.y = cam_angle
	elif ts > 0 && ts < 45 && cam_angle == 180:
		self.rotation_degrees.y = cam_angle
	elif ts == 0 && cam_angle == 180:
		self.rotation_degrees.y = cam_angle


func set_angle_to_fov_curve():
	angle_to_fov.add_point(Vector2(0, 1))
	
	
	angle_to_fov.add_point(Vector2(1, 0))
	

func set_angle_to_offset_curve():
	angle_to_offset.add_point(Vector2(0, 0))
	
	
	angle_to_offset.add_point(Vector2(1, 1))


@export var min_speed:float = 40
@export var min_fov:float = 80
@export var max_speed:float = 220
@export var max_fov:float = 100

func camera_fov(speed, angle):
	return clamp(((((speed - min_speed) / (max_speed - min_speed)) * (max_fov - min_fov) * angle_to_fov.sample(angle)) + min_fov) , min_fov, max_fov)

func camera_offset():
	get_parent().offset2 = offset + Vector3(0, 0, (offester * angle_to_offset.sample(angle_ratio)))
