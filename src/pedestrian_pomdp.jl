
@with_kw mutable struct PedestrianAvoidancePOMDPFrenet{B <: Updater} <: DriverModel{LatLonAccel}
    
    timestep::Float64 = 0.2
    t_current::Float64 = 0
    tick::Int64 = 0


    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)
    a::LatLonAccel = LatLonAccel(0.0, 0)
    desired_velocity::Float64 = 50.0 / 3.6

    obstacles::Vector{ConvexPolygon} = []
    sensor_observations::Vector{Vehicle} = []
    objects_tracked = Dict{Int64, Float64}()

    update_tick_high_level_planner::Int64 = 1

    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams())
    sensor::AutomotiveSensors.GaussianSensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, 0., 0.), 
                                                               AutomotiveSensors.LinearNoise(10, 0., 0.), 0, 0, MersenneTwister(1)) 

    pomdp::SingleOCFPOMDP = SingleOCFPOMDP()
    policy::AlphaVectorPolicy{SingleOCFPOMDP,SingleOCFAction} = AlphaVectorPolicy(pomdp, Vector{Vector{Float64}}())
    updater::B = SingleOCFUpdater(pomdp)

    policy_dec::DecPolicy = DecPolicy(policy, pomdp, policy.action_map, (x,y) -> min.(x,y))



    b::SingleOCFBelief = SingleOCFBelief([],[])
    b_dict::Dict{Int64, SingleOCFBelief} = Dict{Int64, SingleOCFBelief}()

    prediction_obstacle::Array{Float64} = []

    brake_request::Bool = false
    collision_rate::Float64 = 0.0
    risk::Float64 = 0.0
    ttc::Float64 = 0.0
    
end



function AutomotiveDrivingModels.observe!(model::PedestrianAvoidancePOMDPFrenet, scene::Scene, roadway::Roadway, egoid::Int)

    ego = scene[findfirst(egoid, scene)]
    model.pomdp.ego_vehicle = ego
    model.ego_vehicle = ego
    model.sensor_observations = measure(PerfectSensor(), ego, scene, roadway, model.obstacles)

    # add sensor delay 0.2s
    model.sensor_observations = objects_time_delay(model)

#    # get observations from sensor, only visible objects with sensor noise
#    model.sensor_observations = measure(model.sensor, ego, scene, roadway, model.obstacles)
   
    # initialization of the belief for the absent state
    if (model.t_current == 0 )
        model.b_dict[PEDESTRIAN_OFF_KEY] = initBeliefAbsentPedestrian(model.pomdp, ego.state.posF.t, ego.state.v)
    end

    ################ High Level Planner ###################################################
    if (  true ) #model.tick % model.update_tick_high_level_planner == 0 )

        #println("--------------------------POMDP high level planner----------------------- t: ", model.t_current)
        #println("EGO: x/y:", ego.state.posG.x, " / ",  ego.state.posG.y, " v: ", ego.state.v)

        # prepare observation dictionary / remove observations outside state space and add absent state
        observations = get_observations_state_space(model, ego, model.sensor_observations)

        # update belief dictionary
        b_new = update(model.pomdp, model.updater, model.b_dict, SingleOCFAction(model.a.a_lon, model.a.a_lat), observations)
        model.b_dict = deepcopy(b_new)
        #println("dict: ", model.b_dict)
            
b_dict_tmp = Dict{Int64, SingleOCFBelief}()
for oid in keys(model.b_dict)
    if ( oid == PEDESTRIAN_OFF_KEY )
        b_states = []
        b_prob = []
        for (s, prob) in weighted_iterator(model.b_dict[PEDESTRIAN_OFF_KEY] )
            if ( is_state_absent(model.pomdp, s) )
                (ego_y_state_space, ego_v_state_space) = getEgoDataInStateSpace(model.pomdp, ego.state.posF.t, ego.state.v)
                s = SingleOCFState(ego_y_state_space, ego_v_state_space, model.pomdp.S_MAX, 0.0, 0.0 ,0.0)
            end
            push!(b_states, s)
            push!(b_prob, prob)
        end
        normalize!(b_prob, 1)
        b_dict_tmp[PEDESTRIAN_OFF_KEY] =  SingleOCFBelief(b_states, b_prob)  
    else
        b_dict_tmp[oid] = model.b_dict[oid]
    end
end
#println(b_dict_tmp)
act = action(model.policy_dec, b_dict_tmp)

        # use policy and belief dictionary to calculate next action
        act = action(model.policy_dec, model.b_dict)
        model.a = LatLonAccel(act.lateral_movement, act.acc)
        #println("action combined: ", model.a)


   
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


function AutomotiveDrivingModels.propagate(veh::Vehicle, action::LatLonAccel, roadway::Roadway, Δt::Float64)

    # new velocity
    v_ = veh.state.v + action.a_lon*Δt
    v_ = clamp(v_, 0., v_)

    # lateral offset
    delta_y = action.a_lat * Δt   # a_lat corresponds to lateral velocity --> a_lat == v_lat
    if v_ <= 0.
        delta_y = 0.
    end
    y_ = veh.state.posG.y + delta_y
    y_ = clamp(y_, -1.0, 1.0)

    s_new = v_ * Δt

    # longitudional distance based on required velocity and lateral offset
    delta_x = sqrt(s_new^2 - delta_y^2 )
    delta_x = clamp(delta_x, 0., delta_x)
    x_ = veh.state.posG.x + delta_x

    return VehicleState(VecSE2(x_, y_, veh.state.posG.θ), roadway, v_)
end


function AutomotiveDrivingModels.get_name(model::PedestrianAvoidancePOMDPFrenet)
    return "Pedestrian Avoidance System (POMDP)"
end

AutomotiveDrivingModels.rand(model::PedestrianAvoidancePOMDPFrenet) = model.a


@with_kw mutable struct ObservationCallback

    sensor_observations::Vector{Vector{Vehicle}}
    ego_vehicle::Vector{Vehicle}
    ego_a::Vector{Float64}
    collision::Vector{Bool}

    b_dict::Vector{Dict{Int64, SingleOCFBelief}}
    action::Vector{SingleOCFAction}

    collision_rate::Vector{Float64}
    ttc::Vector{Float64}
    risk::Vector{Float64}
    brake_request::Vector{Bool}
    prediction_obstacle::Vector{Array{Float64}}

end


function AutomotiveDrivingModels.run_callback(
        callback::ObservationCallback,
        rec::EntityQueueRecord{S,D,I},
        roadway::R,
        models::Dict{I,M},
        tick::Int) where {S,D,I,R,M<:DriverModel}
    
    push!(callback.sensor_observations, models[1].sensor_observations)
    push!(callback.ego_vehicle, models[1].ego_vehicle)
    push!(callback.ego_a, models[1].a.a_lon)

    collision = is_crash(rec[0])
    push!(callback.collision, collision)

    push!(callback.b_dict, deepcopy(models[1].b_dict))
    push!(callback.action, SingleOCFAction(models[1].a.a_lon, models[1].a.a_lat))

    push!(callback.collision_rate, models[1].collision_rate)
    push!(callback.ttc, models[1].ttc)
    push!(callback.risk, models[1].risk)
    push!(callback.brake_request, models[1].brake_request)
    push!(callback.prediction_obstacle, models[1].prediction_obstacle)

    return collision
end






