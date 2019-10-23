#!/home/hakcar/.local/bin/julia
push!(LOAD_PATH, "/home/hakcar/phd/ml-array-calibration/code-julia/Bayesian-Auto-Calibration/src")

# using BayesianSelfCalibration

println("came here 1")
using RobotOS
@rosimport sensor_msgs.msg: Imu
rostypegen()
using .sensor_msgs.msg

println("came here 2")

cb1(msg::Imu, a::String) = println(a,": ",msg.linear_acceleration.x)
cb2(msg::Imu) = println(msg.angular_velocity.z)
sub1 = Subscriber{Imu}("/imu", cb1, ("accel",), queue_size = 10) #or...
#sub1 = Subscriber("topic", Imu, cb1, ("accel",), queue_size = 10)
sub2 = Subscriber{Imu}("/imu", cb2, queue_size = 10)

println("came here 3")
spin()
