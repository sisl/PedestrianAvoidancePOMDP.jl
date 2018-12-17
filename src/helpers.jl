
function get_observations_state_space(model::FrenetPedestrianPOMDP, ego::Vehicle, sensor_observations::Vector{Vehicle} )

    o = Dict{Int64, SingleOCFObs}()
    for object in sensor_observations
        
        object_posF = Frenet(proj(object.state.posG, get_lane(model.env.roadway, ego.state), 
                                    model.env.roadway, move_along_curves=false), model.env.roadway)
            
        delta_s = object_posF.s - ego.state.posF.s - ego.def.length/2 - object.def.width/2
        delta_t = object_posF.t - ego.state.posF.t
        delta_theta = object_posF.ϕ - ego.state.posF.ϕ
           
         if ( AutomotivePOMDPs.is_observable_fixed(ego.state, VehicleState(VecSE2(ego.state.posG.x+delta_s, ego.state.posG.y+delta_t, 0.), 0.0), model.pomdp.env) == false )
            occluded = true
        else
            occluded = false
        end
        
        if ( occluded == false && delta_s < model.pomdp.S_MAX && delta_s > model.pomdp.S_MIN && delta_t < model.pomdp.T_MAX  &&  delta_t > model.pomdp.T_MIN )
            o[object.id] = SingleOCFState(ego.state.posF.t, ego.state.v, delta_s, delta_t, delta_theta, object.state.v)
            println(o[object.id])    
        end

       # println("PED: t: ", object_posF.t, " / ego: t: ", ego.state.posF.t, " s: ", ego.state.posF.s)
       # println("delta_s: ", delta_s, " delta_t: ", delta_t)
    end
    o[PEDESTRIAN_OFF_KEY] = get_state_absent(model.pomdp, ego.state.posF.t, ego.state.v)
    return o
end

function get_state_absent(pomdp::SingleOCFPOMDP, ego_y::Float64, ego_v::Float64)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, ego_y, ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function get_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    (ego_y_state_space,ego_v_state_space) = getEgoDataInStateSpace(pomdp, s.ego_y, s.ego_v)
    return SingleOCFState(ego_y_state_space, ego_v_state_space, -10.0, -10.0, 0.0 ,0.0)

end

function is_state_absent(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    if (s.ped_s == -10.0 && s.ped_T == -10.0 )
        return true;
    else
        return false;
    end

end

function is_observation_absent(pomdp::SingleOCFPOMDP, o::SingleOCFObs)

    if (o.ped_s > pomdp.S_MAX || o.ped_s < pomdp.S_MIN || o.ped_T > pomdp.T_MAX ||  o.ped_T < pomdp.T_MIN )
        return true;
    else
        return false;
    end

end


function getEgoDataInStateSpace(pomdp::SingleOCFPOMDP, y::Float64, v::Float64)
    y_grid = RectangleGrid(pomdp.EGO_Y_RANGE)
    (id, weight) = interpolants(y_grid, [y])

    ego_y = ind2x(y_grid, findmax(id)[1])[1]
    
    v_grid = RectangleGrid(pomdp.EGO_V_RANGE)
    (id, weight) = interpolants(v_grid, [v])
    ego_v = ind2x(v_grid, findmax(id)[1])[1]
    return ego_y, ego_v
end


function getObstructionCorner(pomdp::SingleOCFPOMDP, obstacle::ConvexPolygon)
 
    x = Vector{Float64}(undef, obstacle.npts)
    y = Vector{Float64}(undef, obstacle.npts)
    for i = 1:obstacle.npts
        x[i] = obstacle.pts[i].x
        y[i] = obstacle.pts[i].y
    end
    
    ego_pos = VecE2(pomdp.ego_vehicle.state.posG.x, pomdp.ego_vehicle.state.posG.y)
    delta_s = maximum(x) - ego_pos.x - pomdp.ego_vehicle.def.length/2
    if delta_s > 0 
        right_side = true
        if ( ego_pos.y > mean(y) )
            delta_t = -(ego_pos.y -  maximum(y))
            right_side = true
        else
            delta_t = minimum(y) - ego_pos.y 
            right_side = false
        end
        return (delta_s, delta_t, right_side) 
    else
        return (1000., 1000., false)
    end
end


function calulateHiddenPositionsRightSide(pomdp::SingleOCFPOMDP, obst_s::Float64, obst_T::Float64)

    sT_pos = []
    if ( obst_s < pomdp.S_MAX)
        idx = findfirst(x -> x >= obst_s, pomdp.S_RANGE)
        s_grid = pomdp.S_RANGE[idx:end]
    
        idx = findlast(x -> x < obst_T, pomdp.T_RANGE)
        T_grid = pomdp.T_RANGE[2:idx]

        thetha = atan(obst_T, obst_s)
        for s in s_grid
            dT = tan(thetha)*(s-obst_s)
            for T in T_grid
                if T < obst_T+dT
                    push!(sT_pos, [s, T])
                end
            end
        end
    end

    return sT_pos
end

function calulateHiddenPositionsLeftSide(pomdp::SingleOCFPOMDP, obst_s::Float64, obst_T::Float64)

    sT_pos = []
    if ( obst_s < pomdp.S_MAX)
        idx = findfirst(x -> x >= obst_s, pomdp.S_RANGE)
        s_grid = pomdp.S_RANGE[idx:end]
    
        idx = findfirst(x -> x >= obst_T, pomdp.T_RANGE)
        T_grid = pomdp.T_RANGE[idx:end-1]
        
        thetha = atan(obst_T, obst_s)
        for s in s_grid
            dT = tan(thetha)*(s-obst_s)
            for T in T_grid
                if T > obst_T+dT
                    push!(sT_pos, [s, T])
                end
            end
        end
    end

    return sT_pos
end
