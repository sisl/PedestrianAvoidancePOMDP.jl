### Transition model  ############################################################################
function POMDPs.transition(pomdp::SingleOCFPOMDP, s::SingleOCFState, a::SingleOCFAction, dt::Float64 = pomdp.ΔT)
    
    ## ego transition 
    # longitudinal motion
    v_ego = s.ego_v + a.acc*pomdp.ΔT 
    sp_ego_v = clamp(v_ego, 0, v_ego)
    x_delta_ego = s.ego_v*pomdp.ΔT + 0.5*a.acc*pomdp.ΔT^2 

    # lateral motion, 0.5m in 0.5s should be fine   
    y_delta_ego = a.lateral_movement * pomdp.ΔT  # simplified movement, not exactly correct -> a=1, t = 1 --> 1m movement to the left
    sp_ego_y = s.ego_y + y_delta_ego
    sp_ego_y = clamp(sp_ego_y, pomdp.EGO_Y_MIN, pomdp.EGO_Y_MAX )

    ## pedestrian transition
    # absent or not
    if ( !is_state_absent(pomdp, s) )   # pedestrian is not absent
        
        states = SingleOCFState[]
        sizehint!(states, 50);
        probs = Float64[] 
        sizehint!(probs, 50);

        # ped transition --> variation in acceleration and theta
        for a_ped in pomdp.PED_A_RANGE
            sp_ped_v = s.ped_v + a_ped * pomdp.ΔT
            sp_ped_v = clamp(sp_ped_v, 0, pomdp.PED_V_MAX)      # speed limitation
            if a_ped == 0           
                a_ped_prob = 3.0
            else
                a_ped_prob = 1.0
            end

            for theta_n in pomdp.PED_THETA_NOISE
                sp_ped_theta = s.ped_theta + theta_n
                @fastmath begin
                sp_ped_s = s.ped_s + sp_ped_v*cos(sp_ped_theta) * pomdp.ΔT - x_delta_ego
                sp_ped_T = s.ped_T + sp_ped_v*sin(sp_ped_theta) * pomdp.ΔT - y_delta_ego  # a_lat corresponds to lateral velocity --> a_lat == v_lat
                end
                sp_ped_s = clamp(sp_ped_s, pomdp.S_MIN, pomdp.S_MAX)
                sp_ped_T = clamp(sp_ped_T, pomdp.T_MIN, pomdp.T_MAX)

           #=
                (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, sp_ego_y, sp_ego_v)
                state_vector = SVector(sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v) # looks faster
                @inbounds ind, weight = interpolants(pomdp.state_space_grid_ped, state_vector)
                for i=1:length(ind)
                    if weight[i] > 0.1
                        state = pomdp.state_space_ped[ind[i]]
                        push!(states, SingleOCFState(ego_y_state_space, ego_v_state_space, state.ped_s, state.ped_T, state.ped_theta, state.ped_v))
                        push!(probs, weight[i]*a_ped_prob)
                    end
                end
=#
                # find the continious state in the state space
                state_vector = SVector(sp_ego_y, sp_ego_v, sp_ped_s, sp_ped_T, sp_ped_theta, sp_ped_v) # looks faster
                @inbounds ind, weight = interpolants(pomdp.state_space_grid, state_vector)
                for i=1:length(ind)
                    if weight[i] > 0.1
                        state = pomdp.state_space[ind[i]]
                        push!(states, state)
                        push!(probs, weight[i]*a_ped_prob)
                    end
                end

            end
        end
    
        # add roughening
        if length(probs) > 1
            normalize!(probs, 1)
            probs .+= maximum(probs)
            normalize!(probs,1)
        end
        return SingleOCFBelief(states,probs)

    else
        # pedestrian is absent
        return initBeliefAbsentPedestrianBorder(pomdp, sp_ego_y, sp_ego_v)
    end

end
