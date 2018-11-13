

#### STATE SPACE
function POMDPs.states(pomdp::SingleOCFPOMDP)
    return pomdp.state_space
end

function POMDPs.stateindex(pomdp::SingleOCFPOMDP, s::SingleOCFState)
    
    if s != pomdp.state_space[end]
        id_tmp = interpolants(pomdp.state_space_grid, [s.ego_y, s.ego_v, s.ped_s, s.ped_T, s.ped_theta, s.ped_v])
        return findmax(id_tmp[1])[1]
    else
        return n_states(pomdp)
    end
end


function POMDPs.n_states(pomdp::SingleOCFPOMDP)
    return length(pomdp.state_space)
end



#### ACTION SPACE
function POMDPs.actions(pomdp::SingleOCFPOMDP)
    return pomdp.action_space
    
end


function POMDPs.n_actions(pomdp::SingleOCFPOMDP)   
  return length(pomdp.action_space)
end


function POMDPs.actionindex(pomdp::SingleOCFPOMDP, a::SingleOCFAction)
    
    lateral_id = 0;
    if a.lateral_movement > 0
        lateral_id = 0
    elseif a.lateral_movement < 0
        lateral_id = 2
    else
        lateral_id = 1
    end
        
    
    if a.acc == 1.
        longitudinal_id = 1
    elseif a.acc == 0.
        longitudinal_id = 2
    elseif a.acc == -1.
        longitudinal_id = 3
    elseif a.acc == -2.
        longitudinal_id = 4    
    else
        longitudinal_id = 5   
    end
        
    if ( length(pomdp.lateral_actions) > 1)
        return longitudinal_id + (lateral_id * length(pomdp.longitudinal_actions))
    else
        return longitudinal_id
    end
end




### OBSERVATION SPACE
function POMDPs.observations(pomdp::SingleOCFPOMDP)
    return states(pomdp)
end


function POMDPs.obsindex(pomdp::SingleOCFPOMDP, o::SingleOCFObs)
    return stateindex(pomdp, o)
end


function POMDPs.n_observations(pomdp::SingleOCFPOMDP)
    return n_states(pomdp)
end
