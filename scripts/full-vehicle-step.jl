using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

t_drive = 1.5
t_steer = 1.5
steer_angle = 0.2
t_end = 24.0

@named model = VehicleComponents.FullVehicleTestStep()
ssys = multibody(model)
prob = ODEProblem(ssys, [
    ssys.steer_angle => steer_angle,
    ssys.throttle.height => 0.1,
    ssys.throttle.duration => 22.5,
    ssys.vehicle.corner_fl.motor.max_torque => 3,
    ssys.vehicle.corner_fr.motor.max_torque => 3,
    ssys.vehicle.corner_rl.motor.max_torque => 3,
    ssys.vehicle.corner_rr.motor.max_torque => 3,
], (0.0, t_end))
sol = solve(prob; tstops=[t_drive, t_steer])

# ===========================================================================
# MoTeC export
# ===========================================================================
VehicleComponents.Telemetry.write_telemetry(
    model, sol;
    run="step_steer",
    vehicle_id=VehicleComponents.params.name,
    venue="Step steer",
    event="FullVehicleTestStep",
    comment="drive at $(t_drive) s, $(steer_angle) rad step at $(t_steer) s"
)

# ===========================================================================
# Animation
# ===========================================================================
cam_offset = GLMakie.Vec3f(-2.5, -0.35, 0.4)
framerate = 25
timevec = range(sol.t[1], sol.t[end], step=1 / framerate)

fig, tobs, scene = render(model, sol, sol.t[1];
    x=cam_offset[1], y=cam_offset[2], z=cam_offset[3],
    up=[0, 0, 1], slider=false, size=(800, 600))

body_r0 = collect(ssys.vehicle.body.frame_a.r_0)
body_R = vec(ssys.vehicle.body.frame_a.R)

GLMakie.record(fig, "output/full_vehicle_step.gif", timevec; framerate) do time
    tobs[] = time

    r0 = sol(time, idxs=body_r0)
    R = reshape(sol(time, idxs=body_R), 3, 3)

    cg_world = MultibodyComponents.resolve1(
        R, VehicleComponents.params.body.cg
    )
    offset_world = MultibodyComponents.resolve1(R, collect(cam_offset))

    target = GLMakie.Vec3f((r0 .+ cg_world)...)
    camera = target + GLMakie.Vec3f(offset_world...)

    GLMakie.update_cam!(
        scene.scene,
        GLMakie.cameracontrols(scene.scene),
        camera,
        target,
        GLMakie.Vec3f(0, 0, 1),
    )
end
