
@with_kw mutable struct FrenetPedestrianPOMDP{B <: Updater} <: DriverModel{LatLonAccel}
    a::LatLonAccel = LatLonAccel(0.0, 0)
    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams())
    sensor::AutomotiveSensors.GaussianSensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, 0., 0.), 
                                                               AutomotiveSensors.LinearNoise(10, 0., 0.), 0, 0, MersenneTwister(1)) 
    timestep::Float64 = 0
    t_current::Float64 = 0
    tick::Int64 = 0
   
    obstacles::Vector{ConvexPolygon}

    risk::Float64 = 0.0
    sensor_observations::Vector{Vehicle} = []


    update_tick_high_level_planner::Int64 = 1

    pomdp::SingleOCFPOMDP = SingleOCFPOMDP()
    policy::AlphaVectorPolicy{SingleOCFPOMDP,SingleOCFAction} = AlphaVectorPolicy(pomdp, Vector{Vector{Float64}}())
    updater::B = SingleOCFUpdater(pomdp)
    b::SingleOCFBelief = SingleOCFBelief() #SingleOCFBelief(Vector{SingleOCFState}(), Vector{Float64}())
    b_dict::Dict{Int64, SingleOCFBelief} = Dict{Int64, SingleOCFBelief}()


    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)

    desired_velocity::Float64 = 40.0 / 3.6

    policy_dec::DecPolicy = DecPolicy(policy, pomdp, policy.action_map, (x,y) -> min.(x,y))
end



function AutomotiveDrivingModels.observe!(model::FrenetPedestrianPOMDP, scene::Scene, roadway::Roadway, egoid::Int)

    ego = scene[findfirst(egoid, scene)]
    model.pomdp.ego_vehicle = ego
    model.ego_vehicle = ego
    sensor = PerfectSensor()
    model.sensor_observations = measure(sensor, ego, scene, roadway, model.obstacles)

#    # get observations from sensor, only visible objects with sensor noise
#    model.sensor_observations = measure(model.sensor, ego, scene, roadway, model.obstacles)
   
    # initialization of the belief for the absent state
    if (model.t_current == 0 )
        model.b_dict[PEDESTRIAN_OFF_KEY] = initBeliefAbsentPedestrian(model.pomdp, ego.state.posF.t, ego.state.v)
    end

    ################ High Level Planner ###################################################
    if (  true ) #model.tick % model.update_tick_high_level_planner == 0 )

        println("--------------------------POMDP high level planner----------------------- t: ", model.t_current)
        println("EGO: x/y:", ego.state.posG.x, " / ",  ego.state.posG.y, " v: ", ego.state.v)

        # prepare observation dictionary / remove observations outside state space and add absent state
        observations = get_observations_state_space(model, ego, model.sensor_observations)

        # update belief dictionary
        b_new = update(model.pomdp, model.updater, model.b_dict, SingleOCFAction(model.a.a_lon, model.a.a_lat), observations)
        model.b_dict = deepcopy(b_new)
        #println("dict: ", model.b_dict)
            
        # use policy and belief dictionary to calculate next action
        act = action(model.policy_dec, model.b_dict)
        model.a = LatLonAccel(act.lateral_movement, act.acc)
        println("action combined: ", model.a)


#=
        # dummy implementation for one belief
        if ( haskey(model.b_dict, 2) )

            model.b = model.b_dict[2]
            
            println("-> perfect observation: ", observations[2])
            obs_state_space = model.pomdp.state_space[stateindex(model.pomdp,observations[2])]
            println("-> perfect observation state space: ", obs_state_space) 
            model.b = SingleOCFBelief([obs_state_space], [1.0])  

            act = action(model.policy, model.b) # policy
            model.a = LatLonAccel(act.lateral_movement, act.acc)
            println("action (ped1): ", model.a ) 
        end
=#
  
        
        # dummy functionality to test transition function / belief update
        #=
        if (model.tick > 2 )
           model.a = LatLonAccel(1.0, -2.0)
            println("manual intervention")
        end
        =#

    end
    model.tick += 1
    model.t_current = model.t_current + model.timestep 
end


# TODO: implementation in Frenet Frame, correction lateral movement into longitudinal velocity
function AutomotiveDrivingModels.propagate(veh::Vehicle, action::LatLonAccel, roadway::Roadway, Δt::Float64)

    # new velocity
    v_ = veh.state.v + action.a_lon*Δt
    v_ = clamp(v_, 0, v_)

    # lateral offset
    delta_y = action.a_lat * Δt   # a_lat corresponds to lateral velocity --> a_lat == v_lat
    if v_ <= 0.
        delta_y = 0.
    end
    s_new = v_ * Δt

    # longitudional distance based on required velocity and lateral offset
#    delta_x = sqrt(s_new^2 - delta_y^2 )
    y_ = veh.state.posG.y + delta_y
    #y_ = clamp(y_, pomdp.EGO_Y_MIN, pomdp.EGO_Y_MAX)
    y_ = clamp(y_, -1.0, 1.0)


    if v_ > 0
        x_ = veh.state.posG.x + veh.state.v*Δt + action.a_lon*Δt^2/2# + delta_x
    else
        x_ = veh.state.posG.x + veh.state.v*Δt# + delta_x
    end

    return VehicleState(VecSE2(x_, y_, veh.state.posG.θ), roadway, v_)
end

function AutomotiveDrivingModels.get_name(model::FrenetPedestrianPOMDP)
    return "Frenet Pedestrian POMDP"
end

AutomotiveDrivingModels.rand(model::FrenetPedestrianPOMDP) = model.a


@with_kw mutable struct ObservationCallback
    risk::Vector{Float64}
    sensor_observations::Vector{Vector{Vehicle}}
    b_dict::Vector{Dict{Int64, SingleOCFBelief}}
    ego_vehicle::Vector{Vehicle}
    action::Vector{SingleOCFAction}
end

function AutomotiveDrivingModels.run_callback(
        callback::ObservationCallback,
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

    return is_crash(rec[0])
end


"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(1, scene)]
    @assert ego.id == 1
    if ego.state.v ≈ 0
        return false
    end
    for veh in scene
        if veh.id != 1
            if AutomotivePOMDPs.is_colliding(ego, veh)
                println(veh)
                println("-----------------> Collision <----------------------")
                return true
            end
        end
    end
    return false
end




