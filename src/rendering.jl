

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
 
             ped = Vehicle(VehicleState(VecSE2(s.ped_s+ego.def.length/2+AutomotivePOMDPs.PEDESTRIAN_DEF.width/2, s.ped_T, s.ped_theta), 0.), AutomotivePOMDPs.PEDESTRIAN_DEF, 1)
             if prob > 1e-6
                 prob_color = RGBA(0.0, 0.0, 1.0, prob*10)
                 add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x, ego.state.posG.y+ped.state.posG.y, ego.state.posG.θ + ped.state.posG.θ, ped.def.length, ped.def.width, prob_color))
             end
         end   
     end
 
     return rendermodel
 end
 


function AutomotivePOMDPs.animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, risk::Vector{Float64}, belief_dict::Vector{Dict{Int64, SingleOCFBelief}}, ego_vehicle::Vector{Vehicle}, action_pomdp::Vector{SingleOCFAction}, prediction::Vector{Array{Float64}}, cam=FitToContentCamera(0.0))

    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)
    function render_rec(t, dt)
       
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", round(ego_vehicle[frame_index].state.v*3.6,digits=1), " km/h" , 
                                " y: ", ego_vehicle[frame_index].state.posG.y, " m",
                                " ax: ", action_pomdp[frame_index].acc, " m/s²")]
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)

        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(80.,10.),incameraframe=true,color=colorant"white",font_size=20)
        belief_overlay = BeliefOverlay(b_dict=belief_dict[frame_index], ego_vehicle=ego_vehicle[frame_index])
      #  max_speed = 14.0
      #  histogram_overlay = HistogramOverlay(pos = VecE2(65.0, 10.0), val=ego_vehicle[frame_index].state.v/max_speed, label="v speed")

        if length(prediction) > 0 
            prediction_overlay = EmergencyBrakingSystem.PretictionOverlay(prediction=prediction[frame_index])
            return AutoViz.render(rec[frame_index-nframes(rec)], env, [prediction_overlay, belief_overlay, sensor_overlay, occlusion_overlay, text_overlay], cam=cam)
        else
            return AutoViz.render(rec[frame_index-nframes(rec)], env, [belief_overlay, sensor_overlay, occlusion_overlay, text_overlay], cam=cam) 
        end
    end
    return duration, fps, render_rec
end
