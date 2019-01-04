#!/bin/bash

# investigation collision speed
#for algorithm in "PedestrianAvoidancePOMDP" 
for algorithm in "PedestrianAvoidancePOMDP_EmergencyBrakingSystem"
do 
    echo "algorithm: " $algorithm


    for keepvelocityreward in 0.0 2.0 4.0 6.0 8.
    do
        echo "keepvelocityreward: " $keepvelocityreward
        for actionloncost in 0.0 -1.0 -2.0 -4.0 
        do
            echo "actionloncost: " $actionloncost
            p_pedestrian = 0.6
            #for p_pedestrian in 0.0 0.2 0.4 0.6 0.8 1.0
            for actionlatcost in -6.0 -8.0  
            do
                echo "p_pedestrian: " $p_pedestrian
                julia1.0 simulate_scenarios.jl --algorithm=$algorithm --policy_name=lateral --pedestrianbirthprobability=$p_pedestrian --actionloncost=$actionloncost --actionlatcost=$actionlatcost --keepvelocityreward=$keepvelocityreward
            done
        done
    done
done

