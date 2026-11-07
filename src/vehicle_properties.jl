"""
    vehicle_mass_and_cg(prob, vehicle)

Total vehicle mass and centre of gravity in its reference geometry, using the
parameters of the simulation problem. This includes sprung mass, uprights,
rims, and tires; wheel centres are the suspension's nominal hardpoints.
"""
function vehicle_mass_and_cg(prob, vehicle)
    mass = prob.ps[vehicle.sprung_mass]
    moment = mass * [prob.ps[vehicle.sprung_cg[i]] for i in 1:3]
    for (corner, center) in (
        (vehicle.corner_fl, vehicle.front_suspension.wheel_center_left),
        (vehicle.corner_fr, vehicle.front_suspension.wheel_center_right),
        (vehicle.corner_rl, vehicle.rear_suspension.wheel_center_left),
        (vehicle.corner_rr, vehicle.rear_suspension.wheel_center_right),
    )
        corner_mass = prob.ps[corner.upright_mass] +
            prob.ps[corner.wheel_assembly.rim_mass] + prob.ps[corner.wheel_assembly.tire_mass]
        moment += corner_mass * [prob.ps[center[i]] for i in 1:3]
        mass += corner_mass
    end
    return mass, moment / mass
end
