#!/bin/bash

# investigation collision speed

collisioncost=-600.0

policy_name="lateral2"

#for algorithm in "PedestrianAvoidancePOMDP"
for algorithm in "PedestrianAvoidancePOMDP_EmergencyBrakingSystem"
do
    echo "algorithm: " $algorithm

    keepvelocityreward=20.0
    actionloncost=-1.0
    for p_pedestrian in 0.0 0.1 0.2 0.3 0.4 0.5 0.6 
    do
        for actionlatcost in 0.0 -10.0 
        do
            for  keeplanereward in 10.0 20.0 50.0 100.0 
            do
                nohup julia1.0 simulate_scenarios.jl --algorithm=$algorithm --policy_name=$policy_name --collisioncost=$collisioncost --pedestrianbirthprobability=$p_pedestrian --actionloncost=$actionloncost --actionlatcost=$actionlatcost --keepvelocityreward=$keepvelocityreward --keeplanereward=$keeplanereward &
            done
        done
    done
done
