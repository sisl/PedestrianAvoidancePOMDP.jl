using Revise
using Distributed

N_PROCS=56
addprocs(N_PROCS)
@everywhere begin 
    using POMDPs
    using GridInterpolations 
    using POMDPModelTools
    using POMDPPolicies

    using Parameters
    using StaticArrays
    using DiscreteValueIteration 

    using AutomotiveDrivingModels
    using AutoViz
    using AutomotiveSensors
    using AutomotivePOMDPs
    using PedestrianAvoidancePOMDP

    pomdp = SingleOCFPOMDP()
end 

solver = ParallelValueIterationSolver(n_procs=N_PROCS, max_iterations=200, belres=1e-4, include_Q=true, verbose=true)

vi_policy = solve(solver, pomdp)
qmdp_policy = AlphaVectorPolicy(pomdp, vi_policy.qmat, vi_policy.action_map)

# save policy!
FileIO.save("policy.jld2", "policy", qmdp_policy)
