using Revise
using AutomotiveDrivingModels
using AutoViz
using AutomotiveSensors
using AutomotivePOMDPs
using GridInterpolations

using POMDPPolicies

using JLD2
using FileIO
using Reel
using Random
using POMDPs
using POMDPModelTools
using LinearAlgebra

using PedestrianAvoidancePOMDP
using EmergencyBrakingSystem






#algorithm = "EmergencyBrakingSystem"
algorithm = "PedestrianAvoidancePOMDP"
algorithm = "PedestrianAvoidancePOMDP_EmergencyBrakingSystem"

policy = load("../policy/policy.jld2")["policy"];

# Definition ego vehicle and pedestrian behavior

scenarios = ["CPCN", "CPAN25", "CPAN75", "CPFA"]
vut_speeds = [10., 15., 20., 25., 30., 35., 40., 45., 50., 55., 60.]
vut_speeds = vut_speeds / 3.6
hit_points = [0., 10., 20., 30., 40., 50.]


ego_v = vut_speeds[9]
hit_point = hit_points[2]
scenario = scenarios[1]

ego_vehicle = []
ego_a = [] 
collision = []
belief = []
action_pomdp = []
collision_rate = []
ttc = []
risk = []
emergency_brake_request = []
prediction_obstacle = []

### Simulate scenario with parameters above defined
# generate scenario based on scenario type
(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, scenario_id) = PedestrianAvoidancePOMDP.generate_scenario(scenario, ego_v, hit_point)
# simulate scenario
if (algorithm == "EmergencyBrakingSystem")
    (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle) = EmergencyBrakingSystem.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles)
else
    (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, belief, action_pomdp, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle) = PedestrianAvoidancePOMDP.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, policy, algorithm)
end
#evaluate result
(collision2, emergency_brake_intervention, dv_collision, v_mean, a_mean, a_jerk, a_min) = PedestrianAvoidancePOMDP.evaluateScenarioMetric(ego_vehicle, emergency_brake_request, ego_a, collision)



# Visualize scenario
if (algorithm == "EmergencyBrakingSystem")
    duration, fps, render_hist = EmergencyBrakingSystem.animate_record(rec, timestep, env, ego_vehicle, sensor, sensor_observations, risk, ttc, collision_rate, emergency_brake_request, prediction_obstacle, StaticCamera(VecE2(38.0,0.0),25.0))
else
    duration, fps, render_hist = animate_record(rec, timestep, env, sensor, sensor_observations, risk, belief, ego_vehicle, action_pomdp, prediction_obstacle, CarFollowCamera(1,10.0))
end

film = roll(render_hist, fps = fps, duration = duration)
#write("file.mp4", film) # Write to a webm video


