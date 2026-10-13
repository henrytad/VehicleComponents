using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots
using ADTypes: AutoFiniteDiff

steer_angle = 0.25
t_steer = 2.0
t_ramp_start = 5.0
ramp_duration = 240.0
v_max = 140 / 3.6
t_end = t_ramp_start + ramp_duration + 5.0

@named model = VehicleComponents.FullVehicleTestConstantSteer()
ssys = multibody(model)
D = Differential(ModelingToolkit.get_iv(ssys))
# TODO: Remove guesses once MTK avoids symbolic guess expansion for linear initialization:
# https://github.com/SciML/ModelingToolkit.jl/issues/5272
prob = ODEProblem(ssys, [
    ssys.steer_angle => steer_angle,
    ssys.steer_start_time => t_steer,
    ssys.ramp_start_time => t_ramp_start,
    ssys.ramp_duration => ramp_duration,
    ssys.v_max => v_max,
], (0.0, t_end); guesses=[D(D(ssys.vehicle.body.r_0[i])) => 0.0 for i in 1:3])
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); tstops=[t_steer, t_ramp_start, t_ramp_start + ramp_duration])

# ===========================================================================
# MoTeC export
# ===========================================================================
VehicleComponents.Telemetry.write_telemetry(
    model, sol;
    rate=200,
    run="constant_steer",
    vehicle_id=VehicleComponents.params.name,
    venue="ISO 4138 constant steer",
    event="FullVehicleTestConstantSteer",
    comment="$(steer_angle) rad held from $(t_steer) s; " *
        "speed ramp 0 to $(round(v_max * 3.6)) km/h over $(ramp_duration) s"
)
