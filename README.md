# PedestrianAvoidancePOMDP

[![Build Status](https://travis-ci.org/sisl/PedestrianAvoidancePOMDP.jl.svg?branch=master)](https://travis-ci.org/sisl/PedestrianAvoidancePOMDP.jl)
[![Coveralls](https://coveralls.io/repos/github/sisl/PedestrianAvoidancePOMDP.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/PedestrianAvoidancePOMDP.jl?branch=master)

## Installation 

```julia 
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoViz.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSensors.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoUrban.jl"))
Pkg.add(PackageSpec(url="https://github.com/JuliaPOMDP/RLInterface.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotivePOMDPs"))
Pkg.add(PackageSpec(url="https://github.com/sisl/GridInterpolations.jl"))
Pkg.add(PackageSpec(url="https://github.com/JuliaPOMDP/POMDPModelTools.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/EmergencyBrakingSystem.jl"))

Pkg.add("POMDPPolicies")
Pkg.add("POMDPs")
using POMDPs
POMDPs.add_registry()
Pkg.add(PackageSpec(url="https://github.com/sisl/PedestrianAvoidancePOMDP.jl"))
```