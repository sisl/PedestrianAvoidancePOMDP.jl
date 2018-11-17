


struct DecPolicy{P <: Policy, M <: Union{MDP, POMDP}, A} <: Policy
    policy::P # the single agent policy
    problem::M # the pomdp definition
    action_map::Vector{A}
    op # the reduction operator for utiliy fusion (e.g. sum or min)
 end

 function POMDPs.action(p::DecPolicy, b::Dict)
    vals = action_values(p, b)
    #ai = indmax(vals)
    #return p.action_map[ai]
    ai = findmax(vals)
  #  println("action_values: ", vals)
  #  println("action_index_max: ", ai)
 #   println("p.action_map[ai[2]]: ", p.action_map[ai[2]])
    #return  findmax(vals)
    return p.action_map[ai[2]]
 end

function action_values(policy::DecPolicy, dec_belief::Dict) 
    return reduce(policy.op, action_values(policy.policy, b) for (_,b) in dec_belief)
 end
 
 function action_values(p::AlphaVectorPolicy, b::SparseCat)
    
    num_vectors = length(p.alphas)
    max_values = -Inf*ones(n_actions(p.pomdp))
    for i = 1:num_vectors
        temp_value = sparse_cat_dot(p.pomdp, p.alphas[i], b)
        ai = actionindex(p.pomdp, p.action_map[i])
        if ( temp_value > max_values[ai])
            max_values[ai] = temp_value
        end

    end
    return max_values

 end
 
 # perform dot product between an alpha vector and a sparse cat object
 function sparse_cat_dot(problem::POMDP, alpha::Vector{Float64}, b::SparseCat)
    val = 0.
    for (s, p) in weighted_iterator(b)
        si = stateindex(problem, s)
        val += alpha[si]*p
    end
    return val
 end


 function AutomotivePOMDPs.action(policy::AlphaVectorPolicy, b::SingleOCFBelief)
    alphas = policy.alphas 
    util = zeros(n_actions(policy.pomdp)) 
    for i=1:n_actions(policy.pomdp)
        res = 0.0
        for (j,s) in enumerate(b.vals)
            si = stateindex(policy.pomdp, s)
            res += alphas[i][si]*b.probs[j]
        end
        util[i] = res
    end
    ihi = findmax(util)[2]
    #println(ihi)
    #println(util)
    #println(policy.action_map[ihi])
    return policy.action_map[ihi]
end