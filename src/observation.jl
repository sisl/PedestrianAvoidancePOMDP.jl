### Observation model  ############################################################################



function observation_weight(pomdp::SingleOCFPOMDP, sp::SingleOCFState, o::SingleOCFObs)

    
    if ( is_observation_absent(pomdp, o) )
        o_absent = true
    else
        o_absent = false
    end

    if (is_state_absent(pomdp, sp) )
        sp_absent = true
    else
        sp_absent = false
    end

   if ( AutomotivePOMDPs.is_observable_fixed(pomdp.ego_vehicle.state, VehicleState(VecSE2(pomdp.ego_vehicle.state.posG.x+sp.ped_s, pomdp.ego_vehicle.state.posG.y+sp.ped_T, 0.), 0.0), pomdp.env) == false )
   # if ( AutomotiveSensors.occlusion_checker(VehicleState(VecSE2(0., 0., 0.), 0.0), VehicleState(VecSE2(sp.ped_s, sp.ped_T, 0.), 0.0), pomdp.env.obstacles) == false )
        occluded = true
    else
        occluded = false
    end


    if ( o_absent && sp_absent )   # absent
        return 1.0
    end

    if ( o_absent && occluded )     # occluded
        return 1.0
    end

    if ( !o_absent && !sp_absent && !o_absent )       # visible

        std_obs = 0.2*eye(4)
        std_obs[3,3] = 0.2        # theta
        std_obs[4,4] = 0.2        # velocity
        ob_dist = MultivariateNormal([sp.ped_s, sp.ped_T, sp.ped_theta ,sp.ped_v], std_obs)

        (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(pomdp, o.ego_y, o.ego_v)

        obs_cont = SVector(ego_y_state_space, ego_v_state_space, o.ped_s, o.ped_T, o.ped_theta, o.ped_v)
        obs_int, obs_weight = interpolants(pomdp.state_space_grid, obs_cont)

        po = 0.0
        N = length(obs_int)
        for i=1:N
            obs_int_state = pomdp.state_space[obs_int[i]]
            po += obs_weight[i]*pdf(ob_dist, [obs_int_state.ped_s, obs_int_state.ped_T, obs_int_state.ped_theta, obs_int_state.ped_v])
        end
        po = po / N
        return po

    else
        return 0.0
    end

end


function POMDPs.observation(pomdp::SingleOCFPOMDP, a::SingleOCFAction, sp::SingleOCFState)
    
    states = SingleOCFObs[]
    sizehint!(states, 100);
    probs = Float64[] 
    sizehint!(probs, 100);
    
    
    push!(states, sp)
    push!(probs, 1)
    
    return SparseCat(states,probs)

end



