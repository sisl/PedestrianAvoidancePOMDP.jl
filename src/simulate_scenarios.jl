
using Revise
using AutomotiveDrivingModels
using AutoViz
using AutomotiveSensors
using AutomotivePOMDPs
using GridInterpolations

using POMDPPolicies
using POMDPModelTools
using DiscreteValueIteration 


using JLD2
using FileIO
using Reel
using Random
using POMDPs
using POMDPModelTools
using LinearAlgebra

using PedestrianAvoidancePOMDP
using EmergencyBrakingSystem


using CSV
using DataFrames


using ArgParse


s = ArgParseSettings()

@add_arg_table s begin

    "--algorithm"
        arg_type = String
        default = "EmergencyBrakingSystem"
        help = "choose among 'EmergencyBrakingSystem', 'PedestrianAvoidancePOMDP', 'PedestrianAvoidancePOMDP_EmergencyBrakingSystem'"
    "--policy_name"
        arg_type = String
        default = "no"
        help = "name for the policy"
    "--collisioncost"
        arg_type = Float64
        default = -500.0
        help = "reward function - cost for collision: -500.0"
    "--actionloncost"
        arg_type = Float64
        default = -1.0
        help = "reward function - cost for longitudinal action: -1.0"
    "--actionlatcost"
        arg_type = Float64
        default = -1.0
        help = "reward function - cost for lateral action: -1.0"
    "--keepvelocityreward"
        arg_type = Float64
        default = 5.0
        help = "reward function - reward for keeping the velocity: 5.0"
    "--pedestrianbirthprobability"
        arg_type = Float64
        default = 0.6
        help = "probability of a new pedestrian behind an obstruction: 0.6"
        
end

parsed_args = parse_args(ARGS, s)

algorithm = parsed_args["algorithm"]
pomdp = SingleOCFPOMDP()

pomdp.COLLISION_COST = parsed_args["collisioncost"]
pomdp.ACTION_LON_COST = parsed_args["actionloncost"]
pomdp.ACTION_LAT_COST = parsed_args["actionlatcost"]
pomdp.KEEP_VELOCITY_REWARD = parsed_args["keepvelocityreward"]
pomdp.KEEP_LANE_REWARD = parsed_args["keeplanereward"]
pomdp.PROBABILITY_PEDESTRIAN_BIRTH = parsed_args["pedestrianbirthprobability"]
pomdp.γ = 0.99

println("algorithm: ", algorithm)

parameters_pomdp = string(pomdp.COLLISION_COST, "_", pomdp.PROBABILITY_PEDESTRIAN_BIRTH, "_", pomdp.ACTION_LON_COST, "_", pomdp.ACTION_LAT_COST, "_", pomdp.KEEP_VELOCITY_REWARD, "_", pomdp.KEEP_LANE_REWARD, "_", pomdp.γ, "_")
policy_name = string(parsed_args["policy_name"], "_", parameters_pomdp, ".jld2")
log_filename = string("results_", algorithm, "_", policy_name, ".csv")


# train policy if required
if algorithm != "EmergencyBrakingSystem"

    println("Policy name: ", policy_name)

    println("pomdp.COLLISION_COST: ", pomdp.COLLISION_COST)
    println("pomdp.ACTION_LON_COST: ", pomdp.ACTION_LON_COST)
    println("pomdp.ACTION_LAT_COST: ", pomdp.ACTION_LAT_COST)
    println("pomdp.KEEP_VELOCITY_REWARD: ", pomdp.KEEP_VELOCITY_REWARD)
    println("pomdp.KEEP_LANE_REWARD: ", pomdp.KEEP_LANE_REWARD)

    println("pomdp.PROBABILITY_PEDESTRIAN_BIRTH: ", pomdp.PROBABILITY_PEDESTRIAN_BIRTH)

    solver = SparseValueIterationSolver(max_iterations=200, belres=1e-4, include_Q=true, verbose=true)
    mdp = UnderlyingMDP(pomdp);

    vi_policy = solve(solver, mdp)
    global policy
    policy = AlphaVectorPolicy(pomdp, vi_policy.qmat, vi_policy.action_map)

    # save policy!
    FileIO.save(string("../policy/", policy_name), "policy", policy)
end


# Evaluate EuroNCAP scenarios

scenarios = ["CPCN", "CPAN25", "CPAN75", "CPFA", "FP"]
vut_speeds = [10., 15., 20., 25., 30., 35., 40., 45., 50., 55., 60.]
vut_speeds = [50.]
vut_speeds = vut_speeds / 3.6


println("Algorithm to evaluate: ", algorithm, " Policy: ", policy_name, " (if specified)")

#rec, timestep, env, ego_vehicle, sensor, sensor_observations, risk, ttc, collision_rate, emergency_brake_request, prediction_obstacle, collision, ego_a
results = Vector[]
for scenario in scenarios
    if (scenario == "FP") 
        hit_points = [-100., 200.]
    else
        hit_points = [0., 10., 20., 30., 40., 50.]
    end
    for hit_point in hit_points
        for vut_speed in vut_speeds
            ego_v = vut_speed
            (ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, scenario_id) = PedestrianAvoidancePOMDP.generate_scenario(scenario, ego_v, hit_point)
            if algorithm == "EmergencyBrakingSystem"
                (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle) = EmergencyBrakingSystem.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles)
            elseif algorithm == "PedestrianAvoidancePOMDP"
                (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, belief, action_pomdp, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle) = PedestrianAvoidancePOMDP.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, policy, algorithm, pomdp.PROBABILITY_PEDESTRIAN_BIRTH)
            elseif algorithm == "PedestrianAvoidancePOMDP_EmergencyBrakingSystem"
                (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, belief, action_pomdp, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle) = PedestrianAvoidancePOMDP.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, policy, algorithm, pomdp.PROBABILITY_PEDESTRIAN_BIRTH)
            else
                println("No valid algorithm defined!")
                return false
            end
            println(scenario, " HP: ", hit_point, " ", ego_x, " ", ego_y, " ", ego_v, " ", ped_x," ",  ped_y," ",  ped_v)
            (collision, emergency_brake_intervention, dv_collision, v_mean, a_mean, a_jerk, a_min) = PedestrianAvoidancePOMDP.evaluateScenarioMetric(ego_vehicle, emergency_brake_request, ego_a, collision, ped_x)
            println("Collision: ", collision, " eb: ", emergency_brake_intervention, " ", dv_collision, " ", v_mean, " ", a_mean, " ", a_jerk, " ", a_min)
            result = [scenario_id, ego_v, hit_point, collision, emergency_brake_intervention, dv_collision, v_mean, a_mean, a_jerk, a_min  ]
            push!(results, result)
        end
    end 
end  


# store results from scneario evaluation in log file
df = DataFrame(results)
df = DataFrame(Matrix(df)')
rename!(df, :x1 => :scenario_id, :x2 => :ego_v, :x3 => :hit_point, :x4 => :collision, :x5 => :eb_intervention)
rename!(df, :x6 => :dv_collision, :x7 => :v_mean, :x8 => :a_mean, :x9 => :a_jerk, :x10 => :a_min)

CSV.write(string("../results/", log_filename), df);

(sum_collisions, sum_eb, dv, v_mean, a_mean, a_jerk, a_min) = PedestrianAvoidancePOMDP.evaluateScenariosMetric(results)

println("#collisions: ", sum_collisions, " #emergency brake interventions: ", sum_eb, " dv_m: ", dv, " v_mean: ", v_mean, " a_mean: ", a_mean, " a_jerk: ", a_jerk, " a_min: ", a_min)
