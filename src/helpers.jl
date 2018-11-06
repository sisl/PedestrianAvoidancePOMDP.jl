
function get_observations_state_space(model::FrenetPedestrianPOMDP, ego::Vehicle, sensor_observations::Vector{Vehicle} )

    o = Dict{Int64, SingleOCFObs}()
    for object in sensor_observations
        println(object)    
        object_posF = Frenet(proj(object.state.posG, get_lane(model.env.roadway, ego.state), 
                                    model.env.roadway, move_along_curves=false), model.env.roadway)
            
        delta_s = object_posF.s - ego.state.posF.s - ego.def.length/2 - object.def.width/2
        delta_t = object_posF.t - ego.state.posF.t
        delta_theta = object_posF.ϕ - ego.state.posF.ϕ
            
        if ( delta_s < pomdp.S_MAX && delta_s > pomdp.S_MIN && delta_t < pomdp.T_MAX  &&  delta_t > pomdp.T_MIN )
            o[object.id] = SingleOCFState(ego.state.posF.t, ego.state.v, delta_s, delta_t, delta_theta, object.state.v)
        end

       # println("PED: t: ", object_posF.t, " / ego: t: ", ego.state.posF.t, " s: ", ego.state.posF.s)
       # println("delta_s: ", delta_s, " delta_t: ", delta_t)
    end
    o[PEDESTRIAN_OFF_KEY] = get_state_absent(pomdp, ego.state.posF.t, ego.state.v)
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
    id_max = indmax(weight)
    ego_y = ind2x(y_grid, id[id_max])[1]
    
    v_grid = RectangleGrid(pomdp.EGO_V_RANGE)
    (id, weight) = interpolants(v_grid, [v])
    id_max = indmax(weight)
    ego_v = ind2x(v_grid, id[id_max])[1]
    return ego_y, ego_v
end


function getObstructionCorner(pomdp::SingleOCFPOMDP, obstacle::ConvexPolygon)
 
    x = Vector{Float64}(obstacle.npts)
    y = Vector{Float64}(obstacle.npts)
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




## Overlay for animation
@with_kw struct BeliefOverlay <: SceneOverlay
   # belief::SingleOCFBelief = SingleOCFBelief()
    b_dict::Dict{Int64, SingleOCFBelief} = Dict{Int64, SingleOCFBelief}()
    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)
    color::Colorant = RGBA(0.0, 0.0, 1.0, 0.2)
end


function AutoViz.render!(rendermodel::RenderModel, overlay::BeliefOverlay, scene::Scene, roadway::R) where R

    b_dict = overlay.b_dict
    ego = overlay.ego_vehicle
    
    for b_id in keys(b_dict)
        for (s, prob) in weighted_iterator( b_dict[b_id])

            ped = Vehicle(VehicleState(VecSE2(s.ped_s+ego.def.length/2+AutomotivePOMDPs.PEDESTRIAN_DEF.width/2, s.ped_T, s.ped_theta), 0.), VehicleDef(AutomotivePOMDPs.PEDESTRIAN_DEF), 1)
            if prob > 1e-6
                prob_color = RGBA(0.0, 0.0, 1.0, prob*10)
                add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x, ego.state.posG.y+ped.state.posG.y, ego.state.posG.θ + ped.state.posG.θ, ped.def.length, ped.def.width, prob_color))
            end
        end   
    end

    return rendermodel
end


function animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, belief_dict::Vector{Dict{Int64, SingleOCFBelief}}, ego_vehicle::Vector{Vehicle}, action_pomdp::Vector{SingleOCFAction}, cam=FitToContentCamera(0.0))
    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
       
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", ego_vehicle[frame_index].state.v, " m/s" , 
                                " y: ", ego_vehicle[frame_index].state.posG.y, " m",
                                " ax: ", action_pomdp[frame_index].acc, " m/s²")]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(20.,10.),incameraframe=true,color=colorant"white",font_size=20)
        belief_overlay = BeliefOverlay(b_dict=belief_dict[frame_index], ego_vehicle=ego_vehicle[frame_index])
      #  max_speed = 14.0
      #  histogram_overlay = HistogramOverlay(pos = VecE2(15.0, 10.0), val=ego_vehicle[frame_index].state.v/max_speed, label="v speed")

        return render(rec[frame_index-nframes(rec)], env, [belief_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end