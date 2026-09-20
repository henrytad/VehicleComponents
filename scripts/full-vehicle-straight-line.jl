# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

t_drive = 1.5
t_end = 5.5

@named model = VehicleComponents.FullVehicleTestStraightLine()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, t_end))
sol = solve(prob; tstops=[t_drive])

# ===========================================================================
# MoTeC export
# ===========================================================================
VehicleComponents.Telemetry.write_telemetry(
    model, sol;
    run="straight_line",
    vehicle_id=VehicleComponents.params.name,
    venue="Straight line",
    event="FullVehicleTestStraightLine",
    comment="drive at $(t_drive) s"
)

# ===========================================================================
# Animation
# ===========================================================================
cam_offset = GLMakie.Vec3f(-2.5, -0.35, 0.4)
framerate = 25
timevec = range(sol.t[1], sol.t[end], step=1 / framerate)

cg = [ssys.vehicle.body.frame_a.r_0[i] + VehicleComponents.params.body.cg[i] for i in 1:3]

fig, tobs, scene = render(model, sol, sol.t[1];
    x=cam_offset[1], y=cam_offset[2], z=cam_offset[3],
    up=[0, 0, 1], slider=false, size=(800, 600))

GLMakie.record(fig, "output/full_vehicle_step.gif", timevec; framerate) do time
    tobs[] = time
    target = GLMakie.Vec3f(sol(time, idxs=cg)...)
    GLMakie.update_cam!(scene.scene, GLMakie.cameracontrols(scene.scene),
        target + cam_offset, target, GLMakie.Vec3f(0, 0, 1))
end
