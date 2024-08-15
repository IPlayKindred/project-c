extends RigidBody3D



#region car data


@export var drivable = true ##true if its the playes car, false if its an AI car

@export_category("power")
@export var power = [0, 300, 600, 650, 670, 705, 705, 690, 0, 0]
@export var rpm = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 7050, 10000]
@export var max_rpm = 9500
@export var min_rpm = 1000
@export var engine_inertia:float = 0.5
@export var shaft_inertia:float = 100

@export_category("transmission")
@export_enum("rwd", "fwd", "awd") var drivetrain = 0
@export var awd_ratio = 0.5
@export var max_gear = 7
@export var gear_ratio = [-4, 0, 3.23, 2.19, 1.71, 1.39, 1.16, 0.93]
@export var final_gear = 4.37
@export var gear_inertia = 0.02

@export_category("automatic setup")
@export var gear_up_rpm = 6000
@export var gear_down_rpm = 4000
@export var slip_threshold = 0.2

@export_category("driving assists")
@export var assist:float = 0.7
@export var min_assist:float = 0.3
@export var speed_assist:float = 0.2

@export var assist_fall_off:float = 10 
@export var max_slip:float = 0.25

@export var max_steering_angle = 30
@export var ackermann_steering = 0.2 
@export var rear_wheel_steering_factor:float = 0




@export_category("brakes")
@export var brake_power:float = 1000
@export var handbrake_power:float = 5000

@export_category("springs")
@export var spring_inwardness:float = 0

@export var spring_len = 0.4
@export var rear_spring_len = 0.4
@export var min_spring_len = 0.4


@export_subgroup("front springs")
@export var spring_force:float = 40000
@export var bump:float = 3000
@export var rebound:float = 3000
@export var anti_roll:float = 1000

@export_subgroup("rear springs")
@export var rear_spring_force:float = 40000
@export var rear_bump:float = 3000
@export var rear_rebound:float = 3000
@export var rear_anti_roll:float = 1000

@export_category("mass and downforce")
@export var coefficient_of_lift:float = 14
@export var downforce_area:float = 2
@export var downforce_ratio = 0.6
@export var coefficient_of_drag = 0.36

@export_category("tyres")
@export var tire_radius = 0.7
@export var tire_stiffness:float = 10
@export var CF:float = 1.0
@export var CFy:float = 1.0
#endregion



@export var check_ray:SpringArm3D


#this is used ot ease in and out between points on the cf graph

var steer_angle = 0

var assist_curve = Curve.new()




var steer = 0
var throttle = 0
var brake = 0
var handbrake = 0.0


var ai_steer = 0


var forward_dir
var forward_steer_dir
var forward_steer_rear_dir
var side_dir
var side_steer_dir
var side_steer_rear_dir




var raycast = []
var springarm = []


var weight_front:float = 0
var weight_rear:float = 0

var dist_to_rear
var weight_ratio

var weight_on_wheels = [0.0, 0.0, 0.0, 0.0]

var air_density = 1.225

var downforce = 0

var forward_speed = 0.0
var side_speed = 0
var steer_speed = 0


var trctr = [0, 0]


var wheel_rpm = 0



@export var mesh:Node
@export var change_the_material = false
var original_mat:Material
@export var change_to_mat:Material

func change_material(change_to:Material):
	if mesh != null && change_the_material:
		for w in mesh.get_children():
			if w is MeshInstance3D:
				if w.mesh.surface_get_material(0) == original_mat:
					w.set_surface_override_material(0, change_to)


var automatic = true

var vvt:bool = false

# Called when the node enters the scene tree for the first time.
func _ready():
	
	#this is neccesary so the raycasts of the raycast of each car dont collide wtih the other car's body.
	#and dont swing the car into the air in seemingly random ways. 
	
	if drivable:
		Car.car = self
	
	Ca = 500000 * tire_stiffness
	Cs = Ca
	
	raycast.append(($"fl"))
	raycast.append(($"fr"))
	raycast.append(($"rl"))
	raycast.append(($"rr"))
	
	
	springarm.append(($"fla"))
	springarm.append(($"fra"))
	springarm.append(($"rla"))
	springarm.append(($"rra"))
	
	
	
	assist_curve.clear_points()
	assist_curve.add_point(Vector2(0, 1)) #segment 0
	assist_curve.add_point(Vector2(assist_fall_off / 90, 0)) #segment 1
	assist_curve.add_point(Vector2(1, 0)) #segment 1
	assist_curve.bake_resolution = 200
	assist_curve.bake()
	
	dist_to_rear = self.center_of_mass.z - (raycast[2].position).z
	weight_ratio = 1 - (dist_to_rear / ((raycast[0].position).z - (raycast[2].position).z))
	
	if drivable == true: 
		drivable = 1
	else: 
		drivable = 0
	


var wgrounded = [0, 0, 0, 0]

var compressions = [0.0, 0.0, 0.0, 0.0]

var distances = [0.0, 0.0, 0.0, 0.0]

var current_spring_len = 0
var current_compression
var current_spring_force

var current_normal

var going_back = 0


var wheel_dir
var trq_on_wheel
var angular_acc_on_wheel
var current_brush

var frc_on_wheel

var wheel_prev_pos = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]
var wheel_speeds = [0.0, 0.0, 0.0, 0.0]
var wheel_speeds_ver = [0.0, 0.0, 0.0, 0.0]
var wheel_speeds_forward = [0.0, 0.0, 0.0, 0.0]
var wheel_speeds_side = [0.0, 0.0, 0.0, 0.0]
var wheel_speeds_prev = [0.0, 0.0, 0.0, 0.0]
var wheel_acc = [0.0, 0.0, 0.0, 0.0]



var side_slip = [0, 0, 0, 0]


var current_wheel_speed
var angle

var slip_interpolation


func ass(input, ackermann):
	return (input + (1 - cos(input * 0.5 * PI)) * ackermann)

var ray_factor = 0

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta) -> void:
	
	procees_steering(delta)
	
	#main stuff
	
	process_vectors(delta)
	
	#slip stuff
	
	process_slip(delta)
	
	#calculate weight and downforce
	
	process_weight(delta)
	
	
	
	process_suspension(delta)
	
	#LATERAL GRIP
	
	for w in [0, 1]:
		current_brush = brushLat(slip_ratio[w], side_slip[w], weight_on_wheels[w], w)
		
		apply_force(side_steer_dir * current_brush * wgrounded[w], (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
	
	for w in [2, 3]:
		current_brush = brushLat(slip_ratio[w], side_slip[w], weight_on_wheels[w], w)
		
		apply_force(side_steer_rear_dir * current_brush * wgrounded[w], (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
	
	#LONG GRIP
	
	for w in [0, 1]:
		current_brush = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w)
		
		apply_force(-forward_steer_dir * current_brush * wgrounded[w], (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
	
	for w in [2, 3]:
		current_brush = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w)
		
		apply_force(-forward_steer_rear_dir * current_brush * wgrounded[w], (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
	
	#power
	
	
	if handbrake == 0 and current_gear != 1:
		clutch = true
	elif handbrake == 1 and current_gear != 1:
		clutch = false
	elif handbrake == 0 and current_gear == 1:
		clutch = false
	elif handbrake == 1 and current_gear == 1:
		clutch = false
	else:
		clutch = true
	
	
	if drivetrain == 0:
		rwd(delta)
	elif drivetrain == 1:
		fwd(delta)
	else:
		awd(delta)
	
	
	
	
	if abs(forward_speed * 2.2) > 0.1:
		wheel_rpm = (wrapf(springarm[0].spin * delta, 0, TAU))
		$"fla/wheel front left".rotate_x(wheel_rpm)
		
		wheel_rpm = (wrapf(springarm[1].spin * delta, 0, TAU))
		$"fra/wheel front right".rotate_x(wheel_rpm)
		
		wheel_rpm = (wrapf(springarm[2].spin * delta, 0, TAU))
		$"rla/wheel rear left".rotate_x(wheel_rpm)
		
		wheel_rpm = (wrapf(springarm[3].spin * delta, 0, TAU))
		$"rra/wheel rear right".rotate_x(wheel_rpm)
	
	if Input.is_action_just_pressed("toggle auto"):
		if automatic == false:
			automatic = true
		else:
			automatic = false
	
	
	if Input.is_action_pressed("handbrake") && drivable == 1: 
		handbrake = 1
	else:
		handbrake = 0
		
		if abs(forward_speed * 2.2) < 2 && (throttle) == 0:
			handbrake = 1
		else:
			handbrake = 0
	
	
	
	if Input.is_action_just_pressed("reset_car") && drivable == 1:
		self.linear_velocity = Vector3.ZERO
		self.angular_velocity = Vector3.ZERO
		for w in range(4):
			springarm[w].spin = 0
			slip_ratio[w] = 0
		
	
	
	if drivable == 1 && automatic == true:
		
		if going_back == 0:
			if forward_speed * 2.2 < 3 && current_gear < 3 && Input.is_action_pressed("brake"):
				going_back = 1
		
		
		
		if Input.is_action_pressed("gas"):
			
			
			if going_back == 1:
				current_gear = 2 
				going_back = 0
			
			throttle = clamp(throttle + 5 * delta, 0, 1)
		else:
			throttle = 0
		
		if Input.is_action_pressed("brake"):
			if going_back == 1:
				current_gear = 0
				throttle = 1
				
			if going_back == 1 && throttle != 0:
				brake = 0
			else:
				brake = 1
		else:
			if going_back == 1:
				throttle = 0 
			else:
				brake = 0
	elif drivable == 1 && automatic == false:
		
		if Input.is_action_pressed("gas"):
			
			throttle = clamp(throttle + 5 * delta, 0, 1)
		else:
			throttle = 0
		
		if Input.is_action_pressed("brake"):
			brake = 1
		else:
			brake = 0
		
	
	
	if Input.is_action_just_pressed("gear_up") && drivable == 1:
		current_gear = clamp(current_gear + 1, 0, max_gear)
	elif Input.is_action_just_pressed("gear_down") && drivable == 1:
		current_gear = clamp(current_gear - 1, 0, max_gear)
	
	if drivable == 0: automatic = true
	
	if engine_rpm > (gear_up_rpm) && current_gear > 0 && automatic == true && ((slip_ratio[2] + slip_ratio[3]) / 2) < slip_threshold && handbrake == 0:
		current_gear += 1
		if current_gear > max_gear: current_gear = max_gear
	elif engine_rpm < (gear_down_rpm) && current_gear > 2 && automatic == true && ((slip_ratio[2] + slip_ratio[3]) / 2) < slip_threshold && handbrake == 0:
		current_gear -= 1
	
	
	
	if engine_rpm > (max_rpm * 0.9): vvt = true


func procees_steering(delta):
	
	if is_instance_valid(check_ray):
		check_ray.position = Vector3.ZERO
		ray_factor = 1 - clamp((check_ray.get_hit_length() / 10), 0, 1)
	
	var speed_interpolation = 0
	speed_interpolation = clamp(((abs(forward_speed) * 2) - 10) / 100, 0, 1)
	
	
	steer += (1.1 - speed_interpolation) * Input.get_axis("steer_left", "steer_right") * delta
	steer = clamp(steer, -1, 1)
	
	
	if Input.get_axis("steer_left", "steer_right") == 0:
		if steer > 0:
			steer -= 3 * delta
		else:
			steer += 3 * delta
	
	slip_interpolation = clamp((forward_speed - 2) / 30, 0, 1)
	
	var assisted_steering = side_slip[2] * 90 * (lerp(min_assist, assist, assist_curve.sample(abs(side_slip[2]))) + (speed_assist * speed_interpolation)) * slip_interpolation
	
	steer_angle = (steer * -max_steering_angle * drivable + assisted_steering) + ai_steer
	
	steer_angle = clamp(steer_angle, -max_steering_angle, max_steering_angle)
	
	springarm[0].rotation_degrees.y = ass(steer_angle, ackermann_steering)
	springarm[1].rotation_degrees.y = ass(steer_angle, -ackermann_steering)
	springarm[2].rotation_degrees.y = ass(-steer_angle * rear_wheel_steering_factor, ackermann_steering)
	springarm[3].rotation_degrees.y = ass(-steer_angle * rear_wheel_steering_factor, -ackermann_steering)





func process_vectors(delta):
	
	forward_dir = global_transform.basis.z
	forward_steer_dir = global_transform.basis.z.rotated(global_transform.basis.y, deg_to_rad(steer_angle))
	forward_steer_rear_dir = global_transform.basis.z.rotated(global_transform.basis.y, deg_to_rad(-steer_angle) * rear_wheel_steering_factor)
	
	
	side_dir = global_transform.basis.x
	side_steer_dir = global_transform.basis.x.rotated(global_transform.basis.y, deg_to_rad(steer_angle))
	side_steer_rear_dir = global_transform.basis.x.rotated(global_transform.basis.y, deg_to_rad(-steer_angle) * rear_wheel_steering_factor)
	wheel_dir = -global_transform.basis.y
	
	
	forward_speed = (linear_velocity.dot(forward_dir))
	side_speed = (linear_velocity.dot(side_dir))
	steer_speed = (linear_velocity.dot(forward_steer_dir))




func process_slip(delta):
	#indivisual raycast slip angle and speeds and slip ratios and traction factors
	
	for w in [0, 1, 2, 3]:
		
		#holyshit, this is such a bad way to figure out if the car is on the road, it forces the map to sperate the road from the road side
		#fuck..
		
		if raycast[w].get_collider(0) && raycast[w].get_collider(0).get_parent().has_method("get_active_material"):
			if raycast[w].get_collider(0).get_parent().get_active_material(0).get_name() == "road":
				traction_factor[w] = 1
			else:
				traction_factor[w] = 1
		else:
			traction_factor[w] = 1
		
		
		current_wheel_speed = ((springarm[w].global_transform.origin - wheel_prev_pos[w]) / delta)
		
		wheel_speeds[w] = current_wheel_speed.length()
		
		if w == 0 or w == 1:
			angle = (asin(current_wheel_speed.normalized().dot(side_steer_dir)))
			
			wheel_speeds_forward[w] = (current_wheel_speed.dot(forward_steer_dir))
			wheel_speeds_side[w] = (current_wheel_speed.dot(side_steer_dir))
		else:
			angle = (asin(current_wheel_speed.normalized().dot(side_dir)))
			
			wheel_speeds_forward[w] = (current_wheel_speed.dot(forward_steer_rear_dir))
			wheel_speeds_side[w] = (current_wheel_speed.dot(side_steer_rear_dir))
		
		side_slip[w] = tan(angle)
		
		
		if not wheel_speeds_forward[w] == 0:  
			slip_ratio[w] = (springarm[w].spin * tire_radius - wheel_speeds_forward[w]) / abs(wheel_speeds_forward[w])
		
		
		wheel_speeds_ver[w] = current_wheel_speed.dot(wheel_dir)
		
		wheel_prev_pos[w] = springarm[w].global_transform.origin
	
	for w in range(0, wheel_acc.size()):
		wheel_acc[w] = (wheel_speeds_ver[w] - wheel_speeds_prev[w])
		wheel_speeds_prev[w] = wheel_speeds_ver[w]




func process_weight(delta):
	downforce = ((air_density * downforce_area * coefficient_of_lift * pow(forward_speed, 2)))
	
	
	
	var drag_force = (air_density * pow(forward_speed, 2) /2) * coefficient_of_drag * downforce_area
	
	apply_central_force(-linear_velocity.normalized() * drag_force)
	
	
	weight_rear = (((self.mass * (9.8))) * weight_ratio) * (1 + cos(deg_to_rad(self.rotation_degrees.x + 90)))
	weight_front = -(((self.mass * (9.8))) - weight_rear)
	
	
	
	weight_front = weight_front + (downforce * (1 - downforce_ratio))
		
	weight_rear = weight_rear + (downforce * downforce_ratio)
	
	#apply downforce and offroad drag
	apply_central_force(wheel_dir * downforce)
	
	var offroad_drag = 0
	
	for w in traction_factor:
		offroad_drag = offroad_drag + w
	
	apply_central_force(-linear_velocity.normalized() * (4 - offroad_drag) * self.mass * 0.2)
	
	#calculating each weight on each raycast
	
	weight_on_wheels[0] = ((weight_front / 2) * (1 + cos(deg_to_rad(raycast[0].global_rotation_degrees.z + 90))))
	weight_on_wheels[1] = weight_front - weight_on_wheels[0]
	weight_on_wheels[2] = ((-weight_rear / 2) * (1 + cos(deg_to_rad(raycast[2].global_rotation_degrees.z + 90))))
	weight_on_wheels[3] = -weight_rear - weight_on_wheels[2]
	
	for w in range(weight_on_wheels.size()):
		weight_on_wheels[w] = weight_on_wheels[w] * ((9.8 + clamp((wheel_acc[w]), -9.8, 9.8)) / 9.8)


func process_suspension(delta):
	#suspension code
	
	
	for w in [0, 1]:
		current_normal = raycast[w].get_collision_normal(0)
		
		
		current_spring_len = springarm[w].get_hit_length()
		
		distances[w] = current_spring_len
		
		if current_spring_len < min_spring_len:
			current_spring_len = clamp(current_spring_len, 0, spring_len)
			current_compression = 1 - (current_spring_len / spring_len)
			current_spring_force = spring_force * current_compression
			
			var antiroll_force
			var antiroll_factor
			
			
			if w == 0:
				antiroll_factor = (current_compression - compressions[1])
				
				antiroll_force = anti_roll * antiroll_factor
				
			else:
				antiroll_factor = (current_compression - compressions[0])
				
				antiroll_force = anti_roll * antiroll_factor
			
			
			current_spring_force += antiroll_force
			
			if (current_compression - compressions[w]) >= 0: current_spring_force += bump * (current_compression - compressions[w]) * 260 
			else: current_spring_force += rebound * (current_compression - compressions[w]) * 260 
			
			if w == 0:apply_force(current_normal.rotated(forward_steer_dir, deg_to_rad(spring_inwardness)) * current_spring_force, (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
			else: apply_force(current_normal.rotated(forward_steer_dir, deg_to_rad(-spring_inwardness)) * current_spring_force, (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
			
			compressions[w] = current_compression
			
			
			wgrounded[w] = 1
		else:
			wgrounded[w] = 0
	
	
	
	for w in [2, 3]:
		
		
		current_normal = raycast[w].get_collision_normal(0)
		
		current_spring_len = springarm[w].get_hit_length()
		
		distances[w] = current_spring_len
		
		if current_spring_len < min_spring_len:
			current_spring_len = clamp(current_spring_len, 0, rear_spring_len)
			current_compression = 1 - (current_spring_len / rear_spring_len)
			current_spring_force = rear_spring_force * current_compression
			
			if w == 2:
				var antiroll_factor = (current_compression - compressions[3])
				
				var antiroll_force = rear_anti_roll * antiroll_factor
				current_spring_force += antiroll_force
			else:
				var antiroll_factor = (current_compression - compressions[2])
				
				var antiroll_force = rear_anti_roll * antiroll_factor
				current_spring_force += antiroll_force
			
			
			if (current_compression - compressions[w]) >= 0: current_spring_force += rear_bump * (current_compression - compressions[w]) * 260
			else: current_spring_force += rear_rebound * (current_compression - compressions[w])  * 260
			
			if w == 2:apply_force(current_normal.rotated(forward_dir, deg_to_rad(spring_inwardness)) * current_spring_force, (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
			else: apply_force(current_normal.rotated(forward_dir, deg_to_rad(-spring_inwardness)) * current_spring_force, (wheel_dir * distances[w]) + raycast[w].global_transform.origin - self.global_transform.origin)
			
			
			compressions[w] = current_compression
			
			
			wgrounded[w] = 1
		else:
			wgrounded[w] = 0
	


var clutch:bool = true

var torque_on_wheel
var spin_on_engine
var trq_from_engine
var current_gear_ratio
var brake_t
var drive_inertia

func rwd(delta):
	current_gear_ratio = (gear_ratio[current_gear] * final_gear)
	
	if slip_ratio[3] != 0 && slip_ratio[2] !=0:
		if abs(slip_ratio[2]) > max_slip && abs(forward_speed) > 4.5: trctr[0] = 0
		else: trctr[0] = 1
		
		if abs(slip_ratio[3]) > max_slip && abs(forward_speed) > 4.5: trctr[1] = 0
		else: trctr[1] = 1
	
	
	
	
	for w in [0, 1]:
		#returns force in newtons
		
		frc_on_wheel = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w) * wgrounded[w]
		
		
		torque_on_wheel = frc_on_wheel * tire_radius
		
		brake_t = ((brake_power / 2) * (brake))
		
		
		springarm[w].process_torque(torque_on_wheel, 19, brake_t, delta)
		
		
	
	
	current_gear = clamp(current_gear, 0, max_gear)
	
	
	
	
	for w in [2, 3]:
		
		
		
		frc_on_wheel = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w) * wgrounded[w]
		
		
		
		if clutch: 
			
			spin_on_engine = (springarm[w].spin * current_gear_ratio)
			engine_rpm = clamp(spin_on_engine * 9.5492968, min_rpm, max_rpm)
			trq_from_engine = gearbox(power_curve(engine_rpm)) * trctr[w - 2] / tire_radius
			
			
			
			torque_on_wheel = ((trq_from_engine / 2) * tire_radius * clamp(throttle, 0, 1)) + (frc_on_wheel * tire_radius) 
			
			
			brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
			
			
			drive_inertia = engine_inertia + shaft_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)
			
		else:
			spin_on_engine = power_curve(clamp(engine_rpm, min_rpm, max_rpm)) * throttle
			
			engine_rpm = clamp(engine_rpm, min_rpm, max_rpm) 
			engine_rpm += spin_on_engine -30
			
			torque_on_wheel = (frc_on_wheel * tire_radius) 
			
			
			brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
			
			
			drive_inertia = engine_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)





func fwd(delta):
	current_gear_ratio = (gear_ratio[current_gear] * final_gear)
	
	if slip_ratio[1] != 0 && slip_ratio[0] !=0:
		if abs(slip_ratio[0]) > max_slip && abs(forward_speed) > 4.5: trctr[0] = 0
		else: trctr[0] = 1
		
		if abs(slip_ratio[1]) > max_slip && abs(forward_speed) > 4.5: trctr[1] = 0
		else: trctr[1] = 1
	
	if trctr[0] == 1 or trctr[1] == 1:
		trctr[0] = 1
		trctr[1] = 1
	
	
	
	for w in [2, 3]:
		#returns force in newtons
		
		frc_on_wheel = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w) * wgrounded[w]
		
		
		torque_on_wheel = frc_on_wheel * tire_radius
		
		
		brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
		
		
		
		springarm[w].process_torque(torque_on_wheel, 4, brake_t, delta)
	
	current_gear = clamp(current_gear, 0, max_gear)
	
	
	
	
	
	for w in [0, 1]:
		
		
		
		
		
		frc_on_wheel = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w) * wgrounded[w]
		
		
		
		if clutch: 
			
			spin_on_engine = (springarm[w].spin * current_gear_ratio)
			engine_rpm = clamp(spin_on_engine * 9.5492968, min_rpm, max_rpm)
			trq_from_engine = gearbox(power_curve(engine_rpm)) * trctr[w - 2] / tire_radius
			
			
			torque_on_wheel = ((trq_from_engine / 2) * tire_radius * clamp(throttle, 0, 1)) + (frc_on_wheel * tire_radius) 
			
			brake_t = ((brake_power / 2) * (brake))
			
			
			drive_inertia = engine_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)
			
		else:
			spin_on_engine = power_curve(clamp(engine_rpm, min_rpm, max_rpm)) * throttle
			
			engine_rpm = clamp(engine_rpm, min_rpm, max_rpm) 
			engine_rpm += spin_on_engine -30
			
			torque_on_wheel = (frc_on_wheel * tire_radius) 
			
			
			brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
			
			
			drive_inertia = engine_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)
			
	


func awd(delta):
	current_gear_ratio = (gear_ratio[current_gear] * final_gear)
	
	if slip_ratio[3] != 0 && slip_ratio[2] !=0:
		if abs(slip_ratio[2]) > max_slip && abs(forward_speed) > 4.5: trctr[0] = 0
		else: trctr[0] = 1
		
		if abs(slip_ratio[3]) > max_slip && abs(forward_speed) > 4.5: trctr[1] = 0
		else: trctr[1] = 1
	
	if trctr[0] == 1 or trctr[1] == 1:
		trctr[0] = 1
		trctr[1] = 1
	
	
	
	
	current_gear = clamp(current_gear, 0, max_gear)
	
	for w in [0, 1, 2, 3]:
		
		
		frc_on_wheel = brushLong(slip_ratio[w], side_slip[w], weight_on_wheels[w], w) * wgrounded[w]
		
		
		
		if clutch: 
			
			spin_on_engine = (springarm[w].spin * current_gear_ratio)
			engine_rpm = clamp(spin_on_engine * 9.5492968, min_rpm, max_rpm)
			trq_from_engine = gearbox(power_curve(engine_rpm)) * trctr[w - 2] / tire_radius
			
			
			torque_on_wheel = ((trq_from_engine / 4) * tire_radius * clamp(throttle, 0, 1)) + (frc_on_wheel * tire_radius) 
			
			
			brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
			
			
			drive_inertia = engine_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)
			
		else:
			spin_on_engine = power_curve(clamp(engine_rpm, min_rpm, max_rpm)) * throttle
			
			engine_rpm = clamp(engine_rpm, min_rpm, max_rpm) 
			engine_rpm += spin_on_engine -30
			
			torque_on_wheel = (frc_on_wheel * tire_radius) 
			
			
			if w > 1:
				brake_t = ((brake_power / 2) * (brake)) + (handbrake_power * handbrake)
			else:
				brake_t = ((brake_power / 2) * (brake))
			
			
			drive_inertia = engine_inertia + pow(abs(current_gear_ratio), 2) * gear_inertia
			
			springarm[w].process_torque(torque_on_wheel, drive_inertia, brake_t, delta)






#brush tyre formula

var Cs = 500000 * tire_stiffness  # longitudinal stiffness of the tyre
var Ca = 500000 * tire_stiffness # cornering stiffness of the tyre

var wav

func wave(longs, lats, Fz, number):
	
	return sqrt(pow((Cs * longs) / (3 * wheel_cf(number) * Fz), 2) + pow((Cs * lats) / (3 * wheel_cf(number) * Fz), 2))

func brushLong(longs, lats, Fz, number):
	if longs == null or lats == null:
		return 0
	
	wav = wave(longs, lats, Fz, number)
	
	if wav < 1: return Cs * longs * pow(1 - wav, 2) + (longs / (sqrt(pow(longs, 2) + pow(lats, 2)))) * wheel_cf(number) * Fz *pow(wav, 2) * (3 - ( 2 * wav))
	else: return (longs / sqrt(pow(longs, 2) + pow(lats, 2))) * wheel_cf(number) * Fz

func brushLat(longs, lats, Fz, number):
	if longs == null or lats == null:
		return 0
	
	wav = wave(longs, lats, Fz, number)
	
	if wav < 1: return Ca * lats * pow(1 - wav, 2) + (lats / (sqrt(pow(longs, 2) + pow(lats, 2)))) * wheel_cfy(number) * Fz * pow(wav, 2) * (3 - ( 2 * wav))
	else: return (lats / sqrt(pow(longs, 2) + pow(lats, 2))) * wheel_cfy(number) * Fz


var traction_factor = [1, 1, 1, 1]

func wheel_cf(number):
	return CF * traction_factor[number]


func wheel_cfy(number):
	return CFy * traction_factor[number]	

var selection = 0


var lerp_diff:float = 0

var lerp_over:float = 0

var ratio:float = 0

func power_curve(input):
	selection = 0
	
	for i in rpm:
		if input >= i:
			selection += 1
		
	
	if selection > 0:
		lerp_diff = rpm[selection] - rpm[selection - 1]
		
		lerp_over = input - rpm[clamp(selection - 1, 0, 100)]
		
		ratio = lerp_over / lerp_diff
		
		return lerp(power[selection - 1], power[selection], ratio)  * 1.25
		
	else:
		return power[0]


var current_gear = 1


func gearbox(in_torque):
	return ((in_torque) * gear_ratio[clamp(current_gear, 0, max_gear)]) * final_gear 



var engine_rpm = 0


var slip_ratio = [0, 0, 0, 0]
