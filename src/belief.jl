
mutable struct SingleOCFUpdater <: Updater
    pomdp::SingleOCFPOMDP
end

const SingleOCFBelief = SparseCat{Vector{SingleOCFState},Vector{Float64}}



function POMDPs.update(pomdp::SingleOCFPOMDP, up::SingleOCFUpdater, bold::Dict{Int64, SingleOCFBelief}, a::SingleOCFAction, o::Dict{Int64, SingleOCFObs})
    bnew = Dict{Int64, SingleOCFBelief}()
    
    for oid in keys(o)

        if haskey(bold, oid) && oid != PEDESTRIAN_OFF_KEY  # old measurment
           @time bnew[oid] = update(up, bold[oid], a, o[oid])

        elseif oid == PEDESTRIAN_OFF_KEY  # absent state

            bnew[PEDESTRIAN_OFF_KEY] = initBeliefAbsentPedestrian(pomdp, o[oid].ego_y, o[oid].ego_v) 
          #  bnew[PEDESTRIAN_OFF_KEY] = update(up, bold[PEDESTRIAN_OFF_KEY], a, o[oid])
           # bnew[PEDESTRIAN_OFF_KEY] = initBeliefAbsentPedestrianBorder(pomdp, o[oid].ego_y, o[oid].ego_v) 

        else # ped appeared
            bnew[oid] = update(up, bold[PEDESTRIAN_OFF_KEY], a, o[oid])
        end
    end

    return bnew
end

function POMDPs.update(up::SingleOCFUpdater, b::SingleOCFBelief, a::SingleOCFAction, o::SingleOCFObs)

    states_p = SingleOCFState[]
    sizehint!(states_p, 1000);

    bp = Float64[] 
    sizehint!(bp, 1000);
    
    
    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

    # object is outside the defined state space
    if ( is_observation_absent(pomdp, o) )
        o = get_state_absent(pomdp,o) 
        #println("Pedestrian is absent: ", o)
    else
       # println("Pedestrian is visible: ", o)
    end

    bp_sum = 0.0   # to normalize the distribution
    for sp_ped in pomdp.state_space_ped

        if ( sp_ped != pomdp.state_space_ped[end])  # not absent state
            sp = SingleOCFState(ego_y_state_space, ego_v_state_space, sp_ped.ped_s, sp_ped.ped_T, sp_ped.ped_theta ,sp_ped.ped_v)
        else # absent state
            sp = get_state_absent(pomdp, ego_y_state_space, ego_v_state_space)
        end
        po = observation_weight(pomdp, sp, o) 


        if po < 0.000000000001
            continue
        end

        b_sum = 0.0   
        for (s, prob) in weighted_iterator(b)
            td = transition(pomdp, s, a) 
            pp = pdf(td, sp)
            b_sum += pp * prob
        end
        
        if b_sum > 0. # && b_sum > 1e-14
            push!(states_p, sp)
            push!(bp, po * b_sum) 
            bp_sum += po * b_sum
        end
 
    end

    if bp_sum == 0.0

        # pedestrian appeared inside the state space, initialize belief with the observation
        if ( !is_observation_absent(pomdp,o) && length(b) == 1)
            for (s,p) in weighted_iterator(b)
                if is_state_absent(pomdp,s) 
                    return initBeliefPedestrian(pomdp, o)
                end
            end
        end
# TODO: not perfect solution
 return initBeliefPedestrian(pomdp, o)        
 
 error("""
              Failed discrete belief update: new probabilities sum to zero.
            #  b = $b
              a = $a
              o = $o
              Failed discrete belief update: new probabilities sum to zero.
              """)
        
    else
        bp ./= bp_sum
    end


    println("b-length: ", length(states_p))
    return SingleOCFBelief(states_p, bp)  

#=
    b_states = []
    b_prob = []
    for (s, prob) in weighted_iterator(SingleOCFBelief(states_p, bp) )
        if ( prob > 1e-4)
            push!(b_states, s)
            push!(b_prob, prob)
        end
    end
    return SingleOCFBelief(b_states, b_prob)  
 =#

end


function POMDPs.initial_state_distribution(pomdp::SingleOCFPOMDP)
    
    bp = ones(n_states(pomdp)) / n_states(pomdp)

    states = SingleOCFState[]
    sizehint!(states, n_states(pomdp))
    
    for state in pomdp.state_space
        push!(states,state)
    end
        
    return SingleOCFBelief(states, bp)  
end



function initBeliefAbsentPedestrianBorder(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)

    states = SingleOCFState[]
    sizehint!(states, 500);
    probs = Float64[] 
    sizehint!(probs, 500);
    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)

    # 
    s_min = pomdp.S_MIN
    if ( pomdp.env.params.obstacles_visible )
        for i = 1:length(pomdp.env.params.obstacles)
            (obst_s, obst_T, right_side) = getObstructionCorner(pomdp, pomdp.env.params.obstacles[i])
            if ( right_side ) 
                s_min = clamp(obst_s, pomdp.S_MIN, pomdp.S_MAX)
            end
        end
    end

    # add states on the right side
    for ped_theta in pomdp.PED_THETA_RANGE
        for ped_v in pomdp.PED_V_RANGE
            for ped_s in pomdp.S_RANGE
                if ( ped_s > s_min)
                    push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, ped_s, pomdp.T_MIN, ped_theta, ped_v))
                end
            end
        end
    end
            
    # add absent state
    absent_state = get_state_absent(pomdp, ego_y_state_space, pomdp.ego_vehicle.state.v)
    push!(states, absent_state)

    probs = ones(length(states))
    probs[1:end - 1] = pomdp.pedestrian_birth / length(states)
    probs[end] = 1.0 - pomdp.pedestrian_birth

    return SingleOCFBelief(states,probs)

end

function initBeliefAbsentPedestrian(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)
    
    states = SingleOCFState[]
    sizehint!(states, 1000);
    probs = Float64[] 
    sizehint!(probs, 1000);

    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)

    # add occluded states
    if ( pomdp.env.params.obstacles_visible )
        for i = 1:length(pomdp.env.params.obstacles)
            ego_pos = VecE2(pomdp.ego_vehicle.state.posG.x, pomdp.ego_vehicle.state.posG.y) 
            #println("ego_pos: ", ego_pos)
            #println("pomdp.env.params.obstacles[i]: ", pomdp.env.params.obstacles[i])

            (obst_s, obst_T, right_side) = getObstructionCorner(pomdp, pomdp.env.params.obstacles[i])
            #println("obst_s: ", obst_s, " obst_T: ", obst_T , " right_side: ", right_side)   
            if ( right_side ) 
                occluded_positions = calulateHiddenPositionsRightSide(pomdp, obst_s, obst_T)

                for ped_theta in pomdp.PED_THETA_RANGE
                    for ped_v = 0.:1.:2.# in pomdp.PED_V_RANGE
                        for (hidden_s, hidden_T) in occluded_positions
                            push!(states,SingleOCFState(ego_y_state_space, ego_v_state_space, hidden_s, hidden_T, ped_theta, ped_v))
                        end
                    end
                end
            end
        end
    end

    # add absent state
    absent_state = get_state_absent(pomdp, ego_y_state_space, ego_v_state_space)
    push!(states,absent_state)

    probs = ones(length(states))
    probs[1:end - 1] .= pomdp.pedestrian_birth / length(states)
    probs[end] = 1.0 - pomdp.pedestrian_birth
  
    return SingleOCFBelief(states,probs)
end

function initBeliefPedestrian(pomdp::SingleOCFPOMDP, o::SingleOCFObs)
    
    states = SingleOCFState[]
    sizehint!(states, 2500);
    probs = Float64[] 
    sizehint!(probs, 2500);

    (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

    for ped_theta in pomdp.PED_THETA_RANGE
        for ped_v in pomdp.PED_V_RANGE
            for ds = -1.0:1.0:1.0
                for dT = -1.0:1.0:1.0
                    obs = SingleOCFState(ego_y_state_space, ego_v_state_space, o.ped_s+ds, o.ped_T+dT, ped_theta, ped_v)
                    obs_int = pomdp.state_space[stateindex(pomdp, obs)]
                    push!(states, obs_int) 
                end
            end
        end
    end
    probs = ones(length(states)) / length(states)

    return SingleOCFBelief(states,probs)
end
