
using Revise
using POMDPs
using GridInterpolations 
using POMDPModelTools
using POMDPPolicies
using Parameters
using StaticArrays
using DiscreteValueIteration 
using Distributed
using AutomotiveDrivingModels
using AutoViz
using AutomotiveSensors
using AutomotivePOMDPs
using PedestrianAvoidancePOMDP

using FileIO
using JLD2

using ArgParse


s = ArgParseSettings()

@add_arg_table s begin
    "--policy_name"
        arg_type = String
        default = "default_name"
        help = "Name of the policy"
    "--policy_type"
        arg_type = String
        default = "longitudinal"
        help = "choose among 'longitudinal', 'lateral'"

end

parsed_args = parse_args(ARGS, s)

policy_name = string(parsed_args["policy_name"], ".jld2")
policy_type = parsed_args["policy_type"]
println("Policy name: ", policy_name)
println("Policy type: ", policy_type)

pomdp = SingleOCFPOMDP()
solver = SparseValueIterationSolver(max_iterations=200, belres=1e-4, include_Q=true, verbose=true)
mdp = UnderlyingMDP(pomdp);

vi_policy = solve(solver, mdp)

qmdp_policy = AlphaVectorPolicy(pomdp, vi_policy.qmat, vi_policy.action_map)

# save policy!
FileIO.save(policy_name, "policy", qmdp_policy)


