#!/bin/bash

# investigation collision speed

collisioncost=-600.0

policy_name="longitudinal"

#for algorithm in "PedestrianAvoidancePOMDP"
for algorithm in "PedestrianAvoidancePOMDP_EmergencyBrakingSystem"
do
    echo "algorithm: " $algorithm

    keeplanereward=0.0
    actionlatcost=0.0
    for keepvelocityreward in 0.0 10.0 20.0 30.0 40.0
    do
        for actionloncost in 0.0 -1.0 -2.0 -5.0
        do
            for p_pedestrian in 0.1 0.3 0.5 0.7 0.9
            do
                julia1.0 simulate_scenarios.jl --algorithm=$algorithm --policy_name=$policy_name --collisioncost=$collisioncost --pedestrianbirthprobability=$p_pedestrian --actionloncost=$actionloncost --actionlatcost=$actionlatcost --keepvelocityreward=$keepvelocityreward --keeplanereward=$keeplanereward
            done
        done
    done
done
