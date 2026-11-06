
using VehicleComponents
using Test
using ADTypes: AutoFiniteDiff
using DyadInterface: ODEAlg
using OrdinaryDiffEqRosenbrock: Rodas5P

function __dyad_run_test_case!(
    rig::Union{typeof(FullVehicleTestStraightLine),typeof(FullVehicleTestConstantSteer),typeof(FullVehicleTestInertiaYaw),typeof(FullVehicleTestInertiaPitch),typeof(FullVehicleTestInertiaRoll)},
    name::AbstractString; solver, kwargs...)
    if solver == ODEAlg.Rodas5P()
        solver = ODEAlg.NativeAlgorithm(Rodas5P(autodiff=AutoFiniteDiff()), @__MODULE__)
    end
    invoke(__dyad_run_test_case!, Tuple{Any,AbstractString}, rig, name; solver, kwargs...)
end

include("damper_maps.jl")
include("../generated/tests.jl")
