
@with_kw mutable struct PedestrianSystem <: DriverModel{LatLonAccel}

    frenet_pedestrian_pomdp::FrenetPedestrianPOMDP{SingleOCFUpdater} = FrenetPedestrianPOMDP()
    emergency_braking_system::EmergencyBrakingSystem.EmergencySystem = EmergencyBrakingSystem.EmergencySystem()

    a::LatLonAccel = LatLonAccel(0.0, 0)

  
    timestep::Float64 = 0
    t_current::Float64 = 0
    tick::Int64 = 0
   
   
   
   
    update_tick_high_level_planner::Int64 = 4
    update_tick_emergency_braking_system::Int64 = 1

    
    b::SingleOCFBelief = SingleOCFBelief()
    b_dict::Dict{Int64, SingleOCFBelief} = Dict{Int64, SingleOCFBelief}()

    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)

    risk::Float64 = 0.0
    sensor_observations::Vector{Vehicle} = []
    prediction::Array{Float64} = []
end



function AutomotiveDrivingModels.observe!(model::PedestrianSystem, scene::Scene, roadway::Roadway, egoid::Int)

    ego = scene[findfirst(egoid, scene)]
    model.ego_vehicle = ego

    println("model.t_current: ", model.t_current)

    ################ Emergency Braking System #############################################
    println("-> emergency braking system")
    observe!(model.emergency_braking_system, scene, roadway, egoid)
    model.prediction = model.emergency_braking_system.prediction

    ################ High Level Planner ###################################################
    if (  model.tick % model.update_tick_high_level_planner == 0 )
        println("--------------------------POMDP high level planner----------------------- t: ", model.t_current)    
   #     observe!(model.frenet_pedestrian_pomdp, scene, roadway, egoid)
   #     model.b_dict = model.frenet_pedestrian_pomdp.b_dict

#        println(model.frenet_pedestrian_pomdp.a)
    end
    

    ################ Combination Emergency System + High Level Planner ####################
    # lateral movement only when no strong decceleration
    if ( model.emergency_braking_system.a.a_lon < -2. )
        a_lat = 0.0
    else
        a_lat = model.frenet_pedestrian_pomdp.a.a_lat
    end
    
    # stronger decceleration is used
    if ( model.emergency_braking_system.a.a_lon < 0) 
        a_lon = min(model.frenet_pedestrian_pomdp.a.a_lon, model.emergency_braking_system.a.a_lon)
    else
       a_lon = model.frenet_pedestrian_pomdp.a.a_lon
    end

    model.a = LatLonAccel(a_lat, a_lon) 
    println("a: ", model.a)

    model.tick += 1
    model.t_current = model.t_current + model.timestep 
end

function AutomotiveDrivingModels.get_name(model::PedestrianSystem)
    return "Pedestrian System"
end

AutomotiveDrivingModels.rand(model::PedestrianSystem) = model.a



@with_kw mutable struct ObservationPedestrianAvoidanceCallback
    risk::Vector{Float64}
    sensor_observations::Vector{Vector{Vehicle}}
    b_dict::Vector{Dict{Int64, SingleOCFBelief}}
    ego_vehicle::Vector{Vehicle}
    action::Vector{SingleOCFAction}
    prediction::Vector{Array{Float64}}
end

function AutomotiveDrivingModels.run_callback(
        callback::ObservationPedestrianAvoidanceCallback,
        rec::EntityQueueRecord{S,D,I},
        roadway::R,
        models::Dict{I,M},
        tick::Int) where {S,D,I,R,M<:DriverModel}
    
    push!(callback.risk, models[1].risk)
    push!(callback.sensor_observations, models[1].sensor_observations)
    push!(callback.b_dict, deepcopy(models[1].b_dict))
    push!(callback.ego_vehicle, models[1].ego_vehicle)
    act = SingleOCFAction(models[1].a.a_lon, models[1].a.a_lat)
    push!(callback.action, act)
    push!(callback.prediction, deepcopy(models[1].prediction))
    return is_crash(rec[0])
end


function AutomotivePOMDPs.animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, belief_dict::Vector{Dict{Int64, SingleOCFBelief}}, ego_vehicle::Vector{Vehicle}, action_pomdp::Vector{SingleOCFAction}, prediction::Vector{Array{Float64}}, cam=FitToContentCamera(0.0))


    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
       
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", ego_vehicle[frame_index].state.v*3.6, " km/h" , 
                                " y: ", ego_vehicle[frame_index].state.posG.y, " m",
                                " ax: ", action_pomdp[frame_index].acc, " m/s²")]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        prediction_overlay = EmergencyBrakingSystem.PretictionOverlay(prediction=prediction[frame_index])

        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(20.,10.),incameraframe=true,color=colorant"white",font_size=20)
        belief_overlay = BeliefOverlay(b_dict=belief_dict[frame_index], ego_vehicle=ego_vehicle[frame_index])
      #  max_speed = 14.0
      #  histogram_overlay = HistogramOverlay(pos = VecE2(15.0, 10.0), val=ego_vehicle[frame_index].state.v/max_speed, label="v speed")

        return AutoViz.render(rec[frame_index-nframes(rec)], env, [prediction_overlay, belief_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end

