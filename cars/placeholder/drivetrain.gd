extends Node3D


@export var wheel_node : Node3D

@export var wheel_mass := 15.0
@export var tire_radius := 0.3
@export var ackermann := 0.15





var wheel_moment := 0.0
var spin := 0.0
var spin_velocity_diff := 0.0
var applied_torque := 0.0
var force_vector := Vector2.ZERO

var steering_ratio := 0.0



var vehicle : Vehicle

func _ready() -> void:
	initialize()

func _physics_process(delta: float) -> void:
	if wheel_node:
		wheel_node.rotation.x -= (wrapf(spin * delta, 0, TAU))
		

func initialize():
	if wheel_node:
		wheel_node.rotation_order = EULER_ORDER_ZXY
	
	wheel_moment = 0.5 * wheel_mass * pow(tire_radius, 2)
	

func steer(input : float, max_steering_angle : float):
	input *= steering_ratio
	rotation.y = (max_steering_angle * (input + (1 - cos(input * 0.5 * PI)) * ackermann))

func process_torque(drive : float, drive_inertia : float, brake_torque : float, delta : float) -> float:
	## Add the torque the wheel produced last frame from surface friction
	var net_torque = force_vector.y * tire_radius
	var previous_spin := spin
	net_torque += drive
	
	
	
	## Applied torque is used to ensure the wheels don't apply more force
	## than the motor or brakes applied to the wheel
	if is_zero_approx(spin):
		applied_torque = absf(drive - brake_torque)
	else:
		applied_torque = absf(drive - (brake_torque * signf(spin)))
	
	## If braking and nearly stopped, just stop the wheel completely.
	if absf(spin) < 5.0 and brake_torque > absf(net_torque):
		spin = 0.0
	else:
		## Spin the wheel based on the provided torque. The tire forces will handle
		## applying that force to the vehicle.
		net_torque -= brake_torque * signf(spin)
		var new_spin : float = spin + ((net_torque / (wheel_moment + drive_inertia)) * delta)
		if signf(spin) != signf(new_spin) and brake_torque > absf(drive):
			new_spin = 0.0
		spin = new_spin
	
	## The returned value is used to track wheel speed difference
	if is_zero_approx(drive * delta):
		return 0.5
	else:
		return (spin - previous_spin) * (wheel_moment + drive_inertia) / (drive * delta)
