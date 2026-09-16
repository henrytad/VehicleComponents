module VehicleComponents

using LinearAlgebra

# Hand-written helpers must come before the generated code that calls them
include("damper_maps.jl")

include("../generated/module.jl")

end # module VehicleComponents