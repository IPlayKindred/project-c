extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready():
	metric = $"metric"

@export var speed:Label3D
@export var race_position:Label3D
@export var distance_left:Label3D
@export var gear:Label3D
@export var gear_indicator:Label3D
@export var rpm:Label3D

@export var left_indicator:Label3D
@export var right_indicator:Label3D
@export var turn_back_indicator:Label3D

var indicator_cycle = 0
var indicator_mid_cycle = 0

var max_angle = 30

var target_position = Vector3.ZERO

var target_angle = 0.0

var metric
var metric_factor = 2.2

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if get_parent().drivable == 1 && get_tree().paused == false:
		self.visible = true
	else:
		self.visible = false
	
	
	if metric.text == "mph":
		metric_factor = 2.2
	else:
		metric_factor = 3.6
	
	
	
	speed.text = str(snapped(abs(Car.car.forward_speed) * metric_factor, 1))
	rpm.text = str(snapped(Car.car.engine_rpm, 100))
	
	
	
	
	
	if Car.car.current_gear == 1:
		gear.text = "neutral"
	elif Car.car.current_gear == 0:
		gear.text = "reverse"
	else: gear.text = "gear " + str(Car.car.current_gear - 1)
	
	if Car.car.automatic == true:
		gear_indicator.text = 'auto'
	else:
		gear_indicator.text = ''
	
	
	if indicator_mid_cycle < 0.5:
		indicator_mid_cycle = indicator_mid_cycle + delta
	else:
		indicator_mid_cycle = 0
		indicator_cycle = indicator_cycle + 1
	
	if indicator_cycle > 3:
		indicator_cycle = 0
	
	if indicator_cycle == 0:
		left_indicator.text = "<"
		right_indicator.text = ">"
	elif indicator_cycle == 1:
		left_indicator.text = "<<"
		right_indicator.text = ">>"
	elif indicator_cycle == 2:
		left_indicator.text = "<<<"
		right_indicator.text = ">>>"
	else:
		left_indicator.text = "<<<<"
		right_indicator.text = ">>>>"
	
	
	
	

var angle:float = 0.0

var right_angle:float = 0.0
var diff:int

func angle_to_vector(sel, vect):
	angle = rad_to_deg(acos(sel.global_transform.basis.z.normalized().dot(vect)))
	
	right_angle = rad_to_deg(acos(sel.global_transform.basis.x.normalized().dot(vect)))
	diff
	
	if right_angle < 90:
		diff = -1
	else:
		diff = 1
	
	
	return angle * diff
