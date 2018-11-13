

N_PROCS=56
addprocs(N_PROCS)
@everywhere begin 
    using POMDPs
    using DiscreteValueIteration 
    
    using AutomotiveDrivingModels
    using AutoViz
    using AutomotiveSensors
    using AutomotivePOMDPs
    using Parameters
    using StaticArrays

    using GridInterpolations 
    using POMDPModelTools
    using POMDPPolicies
    using FileIO
    using JLD2

    
    include("../src/pomdp_types.jl")
    include("../src/spaces.jl")
    include("../src/transition.jl")
    include("../src/observation.jl")
    include("../src/belief.jl")
    include("../src/policy.jl")
    include("../src/frenet_pedestrian_pomdp.jl")
    include("../src/helpers.jl")
    include("../src/rendering.jl")

    include("../src/PedestrianAvoidancePOMDP.jl")
    pomdp = SingleOCFPOMDP()
end 

solver = ParallelValueIterationSolver(n_procs=N_PROCS, max_iterations=30, belres=1e-4, include_Q=true, verbose=true)


vi_policy = solve(solver, pomdp)
qmdp_policy = AlphaVectorPolicy(pomdp, vi_policy.qmat, vi_policy.action_map)

# save policy!
save("policy.jld2", "policy", qmdp_policy)

