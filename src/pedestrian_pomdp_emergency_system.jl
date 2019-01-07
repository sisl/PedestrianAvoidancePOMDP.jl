
@with_kw mutable struct PedestrianAvoidanceSystem <: DriverModel{LatLonAccel}
    timestep::Float64 = 0
    t_current::Float64 = 0
    tick::Int64 = 0
   

    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)
    a::LatLonAccel = LatLonAccel(0.0, 0)

    sensor_observations::Vector{Vehicle} = []

    update_tick_high_level_planner::Int64 = 4
    update_tick_emergency_braking_system::Int64 = 1

    pedestrian_pomdp_frenet::PedestrianAvoidancePOMDPFrenet{SingleOCFUpdater} = PedestrianAvoidancePOMDPFrenet()
    emergency_braking_system::EmergencyBrakingSystem.EmergencySystem = EmergencyBrakingSystem.EmergencySystem()



    b_dict::Dict{Int64, SingleOCFBelief} = Dict{Int64, SingleOCFBelief}()

    prediction_obstacle::Array{Float64} = []

    brake_request::Bool = false
    collision_rate::Float64 = 0.0
    risk::Float64 = 0.0
    ttc::Float64 = 0.0

end




function AutomotiveDrivingModels.observe!(model::PedestrianAvoidanceSystem, scene::Scene, roadway::Roadway, egoid::Int)

    ego = scene[findfirst(egoid, scene)]
    model.ego_vehicle = ego
    
    #println("model.t_current: ", model.t_current)

    ################ Emergency Braking System #############################################
    #println("-> emergency braking system")
    observe!(model.emergency_braking_system, scene, roadway, egoid)
    model.prediction_obstacle = model.emergency_braking_system.prediction_obstacle
    model.brake_request = model.emergency_braking_system.brake_request
    model.ttc = model.emergency_braking_system.ttc
    model.collision_rate = model.emergency_braking_system.collision_rate

    ################ High Level Planner ###################################################
    if (  model.tick % model.update_tick_high_level_planner == 0 )
        #println("--------------------------POMDP high level planner----------------------- t: ", model.t_current)    
        observe!(model.pedestrian_pomdp_frenet, scene, roadway, egoid)
        act = action(model.pedestrian_pomdp_frenet.policy_dec, model.pedestrian_pomdp_frenet.b_dict)
        model.pedestrian_pomdp_frenet.a = LatLonAccel(act.lateral_movement, act.acc)
        # println("Action high-level-planner: ", model.pedestrian_pomdp_frenet.a)
        model.b_dict = model.pedestrian_pomdp_frenet.b_dict
    end
    

    ################ Combination Emergency System + High Level Planner ####################
    # lateral movement only when no strong decceleration
    if ( model.emergency_braking_system.a.a_lon < -2. )
        a_lat = 0.0
    else
        a_lat = model.pedestrian_pomdp_frenet.a.a_lat
    end
    
    # stronger decceleration is used
    if ( model.emergency_braking_system.a.a_lon < 0) 
       # a_lon = min(model.pedestrian_pomdp_frenet.a.a_lon, model.emergency_braking_system.a.a_lon)
        a_lon = model.emergency_braking_system.a.a_lon
    else
       a_lon = model.pedestrian_pomdp_frenet.a.a_lon
    end

    model.a = LatLonAccel(a_lat, a_lon) 
    #println("Action combined: ", model.a)

    model.tick += 1
    model.t_current = model.t_current + model.timestep 
end

function AutomotiveDrivingModels.get_name(model::PedestrianAvoidanceSystem)
    return "Pedestrian Avoidance System (POMDP + Emergency System)"
end

AutomotiveDrivingModels.rand(model::PedestrianAvoidanceSystem) = model.a
