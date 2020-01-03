

"""
Representation of a state for the SingleOCcludedCrosswalk POMDP,
depends on ADM VehicleState type
"""

#### State type

 struct SingleOCFState <: FieldVector{6, Float64}
    ego_y::Float64
    ego_v::Float64    
    ped_s::Float64
    ped_T::Float64
    ped_theta::Float64
    ped_v::Float64
end

struct SingleOCFPedState <: FieldVector{4, Float64}
    ped_s::Float64
    ped_T::Float64
    ped_theta::Float64
    ped_v::Float64
end

#### Action type

 struct SingleOCFAction <: FieldVector{2, Float64}
    acc::Float64
    lateral_movement::Float64
end

#### Observaton type

const SingleOCFObs = SingleOCFState

const PEDESTRIAN_OFF_KEY = -1



#### POMDP type

@with_kw mutable struct SingleOCFPOMDP <: POMDP{SingleOCFState, SingleOCFAction, SingleOCFObs}
    env::CrosswalkEnv = CrosswalkEnv()
    PED_SAFETY_DISTANCE::Float64 = 1.5
    ego_type::VehicleDef = VehicleDef()
    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0 + 2.0*PED_SAFETY_DISTANCE, 1. + 2.0*PED_SAFETY_DISTANCE)
    longitudinal_actions::Vector{Float64} = [1.0, 0.0, -1.0, -2.0, -4.0]
    lateral_actions::Vector{Float64} = [0.0] #[1.0, 0.0, -1.0]
    ΔT::Float64 = 0.2
    PED_A_RANGE::Vector{Float64} = LinRange(-1.0, 1.0, 3)
    PED_THETA_NOISE::Vector{Float64} = LinRange(-0.39/2., 0.39/2., 3)

    EGO_Y_MIN::Float64 = -1.
    EGO_Y_MAX::Float64 = 1.
    EGO_Y_RANGE::Vector{Float64} = LinRange(EGO_Y_MIN, EGO_Y_MAX, 5)

    EGO_V_MIN::Float64 = 0.
    EGO_V_MAX::Float64 = 14.
    EGO_V_RANGE::Vector{Float64} = LinRange(EGO_V_MIN, EGO_V_MAX, 29)

    S_MIN::Float64 = -2.
    S_MAX::Float64 = 50.
    S_RANGE::Vector{Float64} = LinRange(S_MIN, S_MAX, 27)

    T_MIN::Float64 = -5.
    T_MAX::Float64 = 5.
    T_RANGE::Vector{Float64} = LinRange(T_MIN, T_MAX, 11)

    PED_V_MIN::Float64 = 0.
    PED_V_MAX::Float64 = 2.
    PED_V_RANGE::Vector{Float64} = LinRange(PED_V_MIN, PED_V_MAX, 5)

    PED_THETA_MIN::Float64 = 1.57-1.57/2
    PED_THETA_MAX::Float64 = 1.57+1.57/2
    PED_THETA_RANGE::Vector{Float64} = [1.57]# LinRange(PED_THETA_MIN, PED_THETA_MAX, 7)

    
    state_space_grid::GridInterpolations.RectangleGrid = initStateSpace(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)
    state_space::Vector{SingleOCFState} = getStateSpaceVector(state_space_grid)

    state_space_grid_ped::GridInterpolations.RectangleGrid = initStateSpacePed(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)
    state_space_ped::Vector{SingleOCFPedState} = getStateSpacePedVector(state_space_grid_ped)


    action_space::Vector{SingleOCFAction} = initActionSpace(longitudinal_actions, lateral_actions)


    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0.0, 0.0, 0.0), 0.0), VehicleDef(), 1)

    desired_velocity::Float64 = 14. #50.0 / 3.6


    PROBABILITY_PEDESTRIAN_BIRTH::Float64 = 0.6

    COLLISION_COST::Float64 = -500.0
    ACTION_LON_COST::Float64 = -1.0
    ACTION_LAT_COST::Float64 = -1.0
    KEEP_VELOCITY_REWARD::Float64 = 5.0
    KEEP_LANE_REWARD::Float64 = 5.0

    γ::Float64 = 0.95

end


### REWARD MODEL ##################################################################################

function POMDPs.reward(pomdp::SingleOCFPOMDP, s::SingleOCFState, action::SingleOCFAction, sp::SingleOCFState) 
    
    r = 0.

    # is there a collision?
    if collision_checker(pomdp,sp)
        r += (pomdp.COLLISION_COST - sp.ego_v * 20)
        return r
    end
    
    # keep velocity
    if (sp.ego_v <= pomdp.desired_velocity )
        v_factor = (pomdp.desired_velocity-abs(sp.ego_v - pomdp.desired_velocity)) / pomdp.desired_velocity 
        r += (v_factor * pomdp.KEEP_VELOCITY_REWARD)
    else
        r -= pomdp.KEEP_VELOCITY_REWARD
    end

   # stay in center of the road
    if ( abs(sp.ego_y) <= pomdp.EGO_Y_MAX )
        l_factor = 1 - abs(sp.ego_y)  / pomdp.EGO_Y_MAX 
        r += (l_factor * pomdp.KEEP_LANE_REWARD)
    else
        r -= pomdp.KEEP_LANE_REWARD
    end


    # costs for longitudinal actions
    if action.acc > 0. ||  action.acc < 0.0
        r += pomdp.ACTION_LON_COST * abs(action.acc)
    end
 
    # costs for lateral actions
    if abs(action.lateral_movement) > 0 
        r += pomdp.ACTION_LAT_COST * abs(action.lateral_movement) 
    end

    return r
    
end




### HELPERS

function POMDPs.isterminal(pomdp::SingleOCFPOMDP, s::SingleOCFState)

    if collision_checker(pomdp,s)
        return true
    end
    
    return false
end


function POMDPs.discount(pomdp::SingleOCFPOMDP)
    return pomdp.γ
end


function AutomotivePOMDPs.collision_checker(pomdp::SingleOCFPOMDP, s::SingleOCFState)
    
    object_a_def = pomdp.ego_type
    object_b_def = pomdp.ped_type

    center_a = VecSE2(-object_a_def.length/2, s.ego_y, 0.0)
    center_b = VecSE2(s.ped_s, s.ped_T, s.ped_theta)

    # first fast check:
    @fastmath begin
        Δ = sqrt((center_a.x - center_b.x)^2 + (center_a.y - center_b.y)^2)
        r_a = sqrt(object_a_def.length*object_a_def.length/4 + object_a_def.width*object_a_def.width/4)
        r_b = sqrt(object_b_def.length*object_b_def.length/4 + object_b_def.width*object_b_def.width/4)
    end
    if Δ ≤ r_a + r_b
        # fast check is true, run parallel axis theorem
        Pa = AutomotiveDrivingModels.polygon(center_a, object_a_def)
        Pb = AutomotiveDrivingModels.polygon(center_b, object_b_def)
        return AutomotiveDrivingModels.overlap(Pa, Pb)
    end
    return false
end



function initStateSpace(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)
    
    return RectangleGrid(EGO_Y_RANGE, EGO_V_RANGE, S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE) 
end

function initStateSpacePed(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE)

    return RectangleGrid(S_RANGE, T_RANGE, PED_THETA_RANGE, PED_V_RANGE) 

end

function getStateSpacePedVector(ped_grid)

    state_space_ped = SingleOCFPedState[]
        
    for i = 1:length(ped_grid)
        s = ind2x(ped_grid,i)
        push!(state_space_ped,SingleOCFPedState(s[1], s[2], s[3], s[4]))
    end
    # add absent state
    push!(state_space_ped,SingleOCFPedState(-10., -10., 0., 0.))
    return state_space_ped
end

function getStateSpaceVector(grid_space)
    
    state_space = SingleOCFState[]
    
    for i = 1:length(grid_space)
        s = ind2x(grid_space,i)
        push!(state_space,SingleOCFState(s[1], s[2], s[3], s[4], s[5], s[6]))
    end

    # add absent state
    push!(state_space,SingleOCFState(0., 0., -10., -10., 0., 0.))
    return state_space
end


function initActionSpace(longitudinal_actions, lateral_actions)
    
  action_space = SingleOCFAction[]

  for lat_a in lateral_actions
    for lon_a in longitudinal_actions
      push!(action_space, SingleOCFAction(lon_a, lat_a))
    end
  end

  return action_space
    
end
