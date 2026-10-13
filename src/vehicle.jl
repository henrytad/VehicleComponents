"""
    VehicleComponents.Data

Typed description of a car, and the cars themselves. The `.dyad` components read
their defaults from one of these through `VehicleComponents.params`, so a
misspelled or missing value fails when the package loads rather than at model
build.

The types live in their own module because the car has parts that share a name
with the components that model them, `Inboard` and `TireMF61` among them.
"""
module Data

"Left and right values of the same thing."
@kwdef struct Sides{T}
    left::T
    right::T
end

"Rigid-body properties of an upright."
@kwdef struct Upright
    mass::Float64
    i_11::Float64
    i_22::Float64
    i_33::Float64
end

"One corner's linkage hardpoints, in chassis coordinates."
@kwdef struct Linkage
    lca_front::Vector{Float64}
    lca_rear::Vector{Float64}
    lca_outer::Vector{Float64}
    uca_front::Vector{Float64}
    uca_rear::Vector{Float64}
    uca_outer::Vector{Float64}
    tierod_inner::Vector{Float64}
    tierod_outer::Vector{Float64}
    pushrod_outer::Vector{Float64}
    wheel_center::Vector{Float64}
    upright::Upright
end

"Hardpoints of one axle's inboard suspension."
@kwdef struct Inboard
    rocker_pivot::Sides{Vector{Float64}}
    pushrod_inner::Sides{Vector{Float64}}
    heave_pickup::Sides{Vector{Float64}}
    roll_pickup::Sides{Vector{Float64}}
end

"Every hardpoint of one axle."
@kwdef struct Geometry
    inboard::Inboard
    linkages::Sides{Linkage}
end

"Valve code and the two independently adjusted click positions."
@kwdef struct DamperSelection
    valve_code::String
    compression_position::Int
    extension_position::Int
end

"Heave spring and damper of one axle."
@kwdef struct HeaveSetup
    stiffness::Float64
    s_unstretched::Float64
    perch_height::Float64
    damper::DamperSelection
end

"Roll spring and damper of one axle. The pack is double-acting, so it carries no load at `s_neutral`."
@kwdef struct RollSetup
    stiffness::Float64
    s_neutral::Float64
    preload_travel::Float64
    damper::DamperSelection
end

"Static alignment of one corner."
@kwdef struct Alignment
    static_camber::Float64
    static_toe::Float64
end

"Everything adjustable on one axle."
@kwdef struct Setup
    heave::HeaveSetup
    roll::RollSetup
    pushrod_adjust::Sides{Float64}
    alignment::Sides{Alignment}
end

"One axle."
@kwdef struct Axle
    geometry::Geometry
    setup::Setup
end

"Both axles."
@kwdef struct Suspension
    front::Axle
    rear::Axle
end

"Peak force against peak velocity at one spool position, compression positive and extension negative."
@kwdef struct DamperPosition
    compression::Vector{Float64}
    extension::Vector{Float64}
end

"One valve code: velocity breakpoints shared by its spool positions, which are keyed by click."
@kwdef struct DamperTable
    velocity::Vector{Float64}
    positions::Dict{Int,DamperPosition}
end

"Road surface. `roughness` is Gd(n0) at n0 = 0.1 cycles/m, in m^3; 0 gives a smooth road."
@kwdef struct Road
    roughness::Float64
end

"Unsprung rotating mass of one wheel."
@kwdef struct Wheels
    rim_mass::Float64
    rim_inertia::Float64
end

"""
One hub motor.
"""
@kwdef struct Drivetrain
    gear_ratio::Float64
    motor_max_torque::Float64
    motor_max_power::Float64
end

"""
Friction brakes.

`max_torque` is the torque at one corner at full demand and an even split, so
the total across the four corners is `4 * max_torque` whatever the bias.
`bias` is the front share of that total: 0.5 is even, higher moves torque
forward. There is no ABS, so whichever axle is over-braked for its
instantaneous load locks.
"""
@kwdef struct Brakes
    max_torque::Float64
    bias::Float64
end

"Sprung mass and its inertia about the centre of gravity."
@kwdef struct Body
    mass::Float64
    cg::Vector{Float64}
    i_11::Float64
    i_22::Float64
    i_33::Float64
end

"Aerodynamic coefficients and the air they act in."
@kwdef struct Aero
    CdA::Float64
    ClA::Float64
    balance_front::Float64
    rho::Float64
end

"""
Traction and braking slip control settings.
"""
@kwdef struct Control
    slip_target_front::Float64
    slip_target_rear::Float64
    launch_torque::Float64
    slip_loop_gain::Float64
    slip_loop_Ti::Float64
    slip_loop_Ni::Float64
    abs_slip_target::Float64
    abs_gain::Float64
end

"Magic Formula 6.1 coefficients, named as in the tire property file they came from."
@kwdef struct TireMF61
    PROPERTY_FILE_FORMAT::String
    TYRESIDE::String
    UNLOADED_RADIUS::Float64
    WIDTH::Float64
    ASPECT_RATIO::Float64
    RIM_RADIUS::Float64
    RIM_WIDTH::Float64
    BOTTOMING_RADIUS::Float64
    MASS::Float64
    IYY::Float64
    DYNAMIC_STIFFNESS::Float64
    DYNAMIC_DAMPING::Float64
    LONGVL::Float64
    USE_MODE::Float64
    VXLOW::Float64
    INFLPRES::Float64
    NOMPRES::Float64
    VERTICAL_STIFFNESS::Float64
    VERTICAL_DAMPING::Float64
    BREFF::Float64
    DREFF::Float64
    FREFF::Float64
    FNOMIN::Float64
    LONGITUDINAL_STIFFNESS::Float64
    LATERAL_STIFFNESS::Float64
    DAMP_VLOW::Float64
    KPUMIN::Float64
    KPUMAX::Float64
    ALPMIN::Float64
    ALPMAX::Float64
    CAMMIN::Float64
    CAMMAX::Float64
    FZMIN::Float64
    FZMAX::Float64
    LFZO::Float64
    LCX::Float64
    LMUX::Float64
    LEX::Float64
    LKX::Float64
    LHX::Float64
    LVX::Float64
    LCY::Float64
    LMUY::Float64
    LEY::Float64
    LKY::Float64
    LHY::Float64
    LVY::Float64
    LKYC::Float64
    LKZC::Float64
    LMUV::Float64
    LTR::Float64
    LRES::Float64
    LXAL::Float64
    LYKA::Float64
    LVYKA::Float64
    LS::Float64
    LMX::Float64
    LVMX::Float64
    LMY::Float64
    PCX1::Float64
    PDX1::Float64
    PDX2::Float64
    PDX3::Float64
    PEX1::Float64
    PEX2::Float64
    PEX3::Float64
    PEX4::Float64
    PKX1::Float64
    PKX2::Float64
    PKX3::Float64
    PHX1::Float64
    PHX2::Float64
    PVX1::Float64
    PVX2::Float64
    PPX1::Float64
    PPX2::Float64
    PPX3::Float64
    PPX4::Float64
    RBX1::Float64
    RBX2::Float64
    RBX3::Float64
    RCX1::Float64
    REX1::Float64
    REX2::Float64
    RHX1::Float64
    PTX1::Float64
    PTX2::Float64
    PTX3::Float64
    PCY1::Float64
    PDY1::Float64
    PDY2::Float64
    PDY3::Float64
    PEY1::Float64
    PEY2::Float64
    PEY3::Float64
    PEY4::Float64
    PEY5::Float64
    PKY1::Float64
    PKY2::Float64
    PKY3::Float64
    PKY4::Float64
    PKY5::Float64
    PKY6::Float64
    PKY7::Float64
    PHY1::Float64
    PHY2::Float64
    PHY3::Float64
    PVY1::Float64
    PVY2::Float64
    PVY3::Float64
    PVY4::Float64
    PPY1::Float64
    PPY2::Float64
    PPY3::Float64
    PPY4::Float64
    PPY5::Float64
    RBY1::Float64
    RBY2::Float64
    RBY3::Float64
    RBY4::Float64
    RCY1::Float64
    REY1::Float64
    REY2::Float64
    RHY1::Float64
    RHY2::Float64
    RVY1::Float64
    RVY2::Float64
    RVY3::Float64
    RVY4::Float64
    RVY5::Float64
    RVY6::Float64
    PTY1::Float64
    PTY2::Float64
    QSX1::Float64
    QSX2::Float64
    QSX3::Float64
    QSY1::Float64
    QSY2::Float64
    QSY3::Float64
    QSY4::Float64
    QBZ1::Float64
    QBZ2::Float64
    QBZ3::Float64
    QBZ4::Float64
    QBZ5::Float64
    QBZ6::Float64
    QBZ9::Float64
    QBZ10::Float64
    QCZ1::Float64
    QDZ1::Float64
    QDZ2::Float64
    QDZ3::Float64
    QDZ4::Float64
    QDZ6::Float64
    QDZ7::Float64
    QDZ8::Float64
    QDZ9::Float64
    QDZ10::Float64
    QDZ11::Float64
    QEZ1::Float64
    QEZ2::Float64
    QEZ3::Float64
    QEZ4::Float64
    QEZ5::Float64
    QHZ1::Float64
    QHZ2::Float64
    QHZ3::Float64
    QHZ4::Float64
    QPZ1::Float64
    PPZ1::Float64
    PPZ2::Float64
    SSZ1::Float64
    SSZ2::Float64
    SSZ3::Float64
    SSZ4::Float64
    QTZ1::Float64
    MBELT::Float64

end

"A car."
@kwdef struct Vehicle
    name::String
    units::String
    suspension::Suspension
    road::Road
    dampers::Dict{String,DamperTable}
    tires::TireMF61
    wheels::Wheels
    drivetrain::Drivetrain
    brakes::Brakes
    body::Body
    aero::Aero
    control::Control
end

"""
    wheelbase(vehicle)

Front axle to rear axle, taken from the same wheel centres the linkages are built
from. Nothing in the model ties the aero centre of pressure to the suspension
geometry, so deriving it here keeps the two from drifting apart.
"""
function wheelbase(vehicle::Vehicle)
    front, rear = vehicle.suspension.front, vehicle.suspension.rear
    front.geometry.linkages.left.wheel_center[1] == front.geometry.linkages.right.wheel_center[1] ||
        throw(ArgumentError("front wheel centres disagree on x"))
    rear.geometry.linkages.left.wheel_center[1] == rear.geometry.linkages.right.wheel_center[1] ||
        throw(ArgumentError("rear wheel centres disagree on x"))
    return front.geometry.linkages.left.wheel_center[1] - rear.geometry.linkages.left.wheel_center[1]
end

# include("vehicles/MR25.jl")
include("vehicles/TestVehicle.jl")

end # module Data
