

function generate_scenario(scenario, ego_v, hit_point)
    
    ego_y = 0.0
    ped_x = 100.0
    ped_y = 0.0
    
    if scenario == "CPCN"
        scenario_id = 1
        ped_v = 5/3.6
        obstacles = [ConvexPolygon([VecE2(ped_x-VehicleDef().length-1, ped_y-2), 
                VecE2(ped_x-VehicleDef().length-1, ped_y-2-VehicleDef().width), 
                VecE2(ped_x-1, ped_y-2-VehicleDef().width), 
                VecE2(ped_x-1, ped_y-2)],4)]
        
    elseif scenario == "CPAN25"
        scenario_id = 2
        ped_v = 5 /3.6
        obstacles = []   
        
    elseif scenario == "CPAN75"
        scenario_id = 3
        ped_v = 5 /3.6
        hit_point = hit_point + 50
        obstacles = []    
        
    elseif scenario == "CPFA"
        scenario_id = 4
        ped_v = 8 / 3.6
        obstacles = [] 

    elseif scenario == "FP"
        scenario_id = 9
        ped_v = 5 /3.6
        obstacles = []  
    else
        scenario_id = -1
        ped_v = 5 / 3.6
        obstacles = []
    end
    
    # fix values
    ped_right_side = true
    ped_theta = 1.5707
    ped_t_collision = 3.0

    ped_y_end = VehicleDef().width * hit_point / 100 - VehicleDef().width/2
    if (ped_right_side)
        ped_y_start = ped_y_end - ped_t_collision * ped_v
    else
        ped_y_start = ped_y_end + ped_t_collision * ped_v
    end
    ped_y = ped_y_start
    ego_x = ped_x - ego_v * ped_t_collision - VehicleDef().length/2;
    #println("Scenario: ", scenario, " v_ego=", ego_v*3.6, "km/h v_ped=", ped_v*3.6, "km/h HP=", hit_point, " ped_y: ", ped_y)
    return (ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, scenario_id)
end


function evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles, policy, system, probability_pedestrian_birth)

    timestep = 0.05
    timestep_pomdp = 0.2
    AX_MAX = -10.

    params = CrosswalkParams()    
    if (length(obstacles) > 0)
        params.obstacles = obstacles
        params.obstacles_visible = true
    else
        params.obstacles_visible = false
    end
    params.roadway_length = 400.0

    env = CrosswalkEnv(params);


    ego_id = 1
    ped_id = 2
    ped2_id = 3
    ped3_id = 4

    # Car definition
    ego_initial_state = VehicleState(VecSE2(ego_x, ego_y, 0.), env.roadway.segments[1].lanes[1], env.roadway, ego_v)
    ego = Vehicle(ego_initial_state, VehicleDef(), ego_id)

    # Pedestrian definition using our new Vehicle definition
    ped_initial_state = VehicleState(VecSE2(ped_x, ped_y, ped_theta), env.crosswalk, env.roadway, ped_v)
    ped = Vehicle(ped_initial_state, AutomotivePOMDPs.PEDESTRIAN_DEF, ped_id)

    ped2 = Vehicle(VehicleState(VecSE2(90., 5., -1.57), env.crosswalk, env.roadway, 0.), AutomotivePOMDPs.PEDESTRIAN_DEF, ped2_id)
    ped3 = Vehicle(VehicleState(VecSE2(103., 5., -1.57), env.crosswalk, env.roadway, 1.), AutomotivePOMDPs.PEDESTRIAN_DEF, ped3_id)

    scene = Scene()
    push!(scene, ego)
    push!(scene, ped)
    #push!(scene, ped2)
    #push!(scene, ped3)

    pos_noise = 0.0
    vel_noise = 0.0
    false_positive_rate = 0.0
    false_negative_rate = 0.0
    rng = MersenneTwister(1);
    sensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, pos_noise, 0.0), 
                     AutomotiveSensors.LinearNoise(10, vel_noise, 0.0), false_positive_rate, false_negative_rate, rng) 


    pomdp = SingleOCFPOMDP()
    pomdp.env = env
    pomdp.desired_velocity = ego_v
    pomdp.ΔT = timestep_pomdp
    pomdp.PROBABILITY_PEDESTRIAN_BIRTH = probability_pedestrian_birth
    updater = SingleOCFUpdater(pomdp)

    # define a model for each entities present in the scene
    models = Dict{Int, DriverModel}()

    # definition pedestrian avoidance POMDP 
    pedestrian_pomdp_frenet = PedestrianAvoidancePOMDPFrenet(a=LatLonAccel(0.0, 0.0),
        env=env,
        sensor=sensor,
        obstacles=env.obstacles,
        timestep=timestep_pomdp,
        update_tick_high_level_planner = timestep_pomdp / timestep,
        pomdp=pomdp,
        policy=policy,
        updater=updater,
        ego_vehicle=ego,
        desired_velocity=ego_v,
        b=PedestrianAvoidancePOMDP.initBeliefAbsentPedestrian(pomdp, ego_y, ego_v)
    )

    if (system == "PedestrianAvoidancePOMDP")
        println("PedestrianAvoidancePOMDP")
        models[ego_id] = pedestrian_pomdp_frenet

    else
        println("PedestrianAvoidancePOMDP_EmergencyBrakingSystem")
        # definition emergency braking system
        emergency_braking_system = EmergencyBrakingSystem.EmergencySystem(
            a=LatLonAccel(0.0, 0.0),
            env=env,
            sensor=sensor, 
            obstacles=env.obstacles,
            timestep=timestep, 
            SAFETY_DISTANCE_LON=1.5,
            AX_MAX=AX_MAX,
            THRESHOLD_COLLISION_RATE=0.5,
            THRESHOLD_TIME_TO_REACT=0.99
        )

        # definition of the pedestrian avoidance system (pomdp + emergency braking system)
        models[ego_id] = PedestrianAvoidanceSystem(
            pedestrian_pomdp_frenet=pedestrian_pomdp_frenet,
            emergency_braking_system=emergency_braking_system,
            timestep=timestep,
            update_tick_high_level_planner=timestep_pomdp / timestep,
        )

    end
    models[ped_id] = ConstantPedestrian(v_desired=ped_v, dawdling_amp=0.0) # dumb model
   # models[ped2_id] = ConstantPedestrian(v_desired=0.0, dawdling_amp=0.05) # dumb model
   # models[ped3_id] = ConstantPedestrian(v_desired=0.0, dawdling_amp=0.05) # dumb model

    nticks = 160
    rec = SceneRecord(nticks+1, timestep)



    # callback data definition
    sensor_observations = [Vehicle[]]
    ego_vehicle = Vehicle[]
    ego_a = Float64[]
    collision = Bool[]
       
    belief = Dict{Int64, SingleOCFBelief}[]
    action_pomdp = SingleOCFAction[]
       
    collision_rate = Float64[]  
    ttc = Float64[]             
    risk = Float64[]
    emergency_brake_request = Bool[]      
    prediction_obstacle = Vector{Array{Float64}}()  

    obs_callback = (ObservationCallback(sensor_observations, ego_vehicle, 
                        ego_a, collision, belief, action_pomdp, collision_rate, ttc, 
                        risk, emergency_brake_request,prediction_obstacle),)

    simulate!(rec, scene, env.roadway, models, nticks, obs_callback)

    return (rec, timestep, env, sensor, sensor_observations, ego_vehicle, ego_a, collision, belief, action_pomdp, collision_rate, ttc, risk, emergency_brake_request, prediction_obstacle)

end

function evaluateScenarioMetric(ego_vehicle, emergency_brake_request, ego_a, collision_vector, ped_x)

    emergency_brake_intervention = false
    collision = false
    dv_collision = 0.
    
    v = []
    a = []
    a_jerk = 0.
    a_last = 0.
    for i=1:length(ego_vehicle)
        # do not consider acceleration part, only until the pedestrian is reached
        if (ego_vehicle[i].state.v > 0 && ego_vehicle[i].state.posG.x < ped_x - 3 )
        #if (ego_vehicle[i].state.v > 0)
            push!(v,ego_vehicle[i].state.v)
        end
        
        if (ego_a[i] != 0. && ego_a[i] < 0.)
            push!(a,ego_a[i])
        end   
        
        if ( a_last != ego_a[i] )
            a_jerk = a_jerk + abs(ego_a[i] - a_last) 
        end
        a_last = ego_a[i]

        if length(emergency_brake_request) > 0 && emergency_brake_request[i] == 1  
            emergency_brake_intervention = true
        end
        
    end
    v_mean = mean(v)
    if ( length(a) > 0)
        a_mean = mean(a)
        a_min = minimum(a)
    else
        a_mean = 0.
        a_min = 0.
    end
    
    if (collision_vector[end] == true && ego_vehicle[end].state.posG.x < ped_x - 2.)   # ignore collisions when the pedestrian runs into the side of the car (ego vehicle too slow)
        collision = true
        dv_collision = ego_vehicle[end].state.v
    end
    
    return (collision, emergency_brake_intervention, dv_collision, v_mean, a_mean, a_jerk, a_min)
end

function evaluateScenariosMetric(results)
    
    sum_collisions = 0
    sum_eb = 0

    dv = []
    v_mean = []
    a_mean = []
    a_jerk = []
    a_min = []

    for i=1:length(results)
        #println(results[i])
        if ( results[i][4] == 1.0 )
            sum_collisions += 1
        end
        if ( results[i][5] == 1.0 )
            sum_eb += 1
        end

        push!(dv,results[i][6])
        push!(v_mean,results[i][7])
        push!(a_mean,results[i][8])
        push!(a_jerk,results[i][9])
        push!(a_min,results[i][10])

    end
    dv = mean(dv)
    v_mean = mean(v_mean)
    a_mean = mean(a_mean)
    a_jerk = mean(a_jerk)
    a_min = mean(a_min)

    return (sum_collisions, sum_eb, dv, v_mean, a_mean, a_jerk, a_min)
end
