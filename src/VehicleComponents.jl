module VehicleComponents

using LinearAlgebra

# Hand-written helpers must come before the generated code that calls them
include("vehicle.jl")
include("damper_maps.jl")
include("motec.jl")
include("telemetry.jl")
include("road_surface.jl")

"The car whose values the `.dyad` component defaults read."
params = Data.VEHICLE

include("../generated/module.jl")

end # module VehicleComponents