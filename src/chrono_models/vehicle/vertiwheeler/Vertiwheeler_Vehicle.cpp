// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Asher Elmquist, Jayne Henry
// =============================================================================
//
// Vertiwheeler full vehicle model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{
Vertiwheeler_Vehicle::Vertiwheeler_Vehicle(const bool fixed,
                             ChContactMethod contact_method,
                             CollisionType chassis_collision_type)
    : ChWheeledVehicle("Vertiwheeler", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, chassis_collision_type);
}

Vertiwheeler_Vehicle::Vertiwheeler_Vehicle(ChSystem* system, const bool fixed, CollisionType chassis_collision_type)
    : ChWheeledVehicle("Vertiwheeler", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, chassis_collision_type);
}


void Vertiwheeler_Vehicle::Create(bool fixed, CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Vertiwheeler_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();
    
    m_axles[0]->m_suspension = chrono_types::make_shared<Vertiwheeler_BellcrankSolid3LinkAxle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Vertiwheeler_Solid3LinkAxle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Vertiwheeler_WheelLeft>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Vertiwheeler_WheelLeft>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Vertiwheeler_WheelLeft>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Vertiwheeler_WheelLeft>("Wheel_RR");

    m_axles[0]->m_brake_left = chrono_types::make_shared<Vertiwheeler_BrakeSimple>("Brake_FL");
    m_axles[0]->m_brake_right = chrono_types::make_shared<Vertiwheeler_BrakeSimple>("Brake_FR");
    m_axles[1]->m_brake_left = chrono_types::make_shared<Vertiwheeler_BrakeSimple>("Brake_RL");
    m_axles[1]->m_brake_right = chrono_types::make_shared<Vertiwheeler_BrakeSimple>("Brake_RR");

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Vertiwheeler_RotaryArm>("Steering");


    // Create the driveline
    m_driveline = chrono_types::make_shared<Vertiwheeler_Driveline4WD>("Driveline");
}

Vertiwheeler_Vehicle::~Vertiwheeler_Vehicle() {}

void Vertiwheeler_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // ORIGIN WILL BE LOCATED AT THE CENTER OF THE FRONT AXLE

    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector<> offset = ChVector<>(0, 0, 0);
    ChQuaternion<> rotation = ChQuaternion<>(1, 0, 0, 0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector<>(0.153, 0, -0.012), ChVector<>(1,0,0), 0.0, m_omega[0],m_omega[1]);

    //4TH ARGUMENT IS THE OFFSET OF THE WHEEL FROM THE CENTER OF THE CHASSIS
    m_axles[1]->Initialize(m_chassis, nullptr,nullptr, ChVector<>(-0.153+(-0.005), 0, -0.012), ChVector<>(0), 0.0, m_omega[2], m_omega[3]);
    std::vector<int> driven_susp_indexes = {0, 1};
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}


void Vertiwheeler_Vehicle::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidBellcrankThreeLinkAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChSolidThreeLinkAxle>(m_axles[1]->m_suspension)
        ->LogHardpointLocations(ChVector<>(0, 0, 0), false);

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------

void Vertiwheeler_Vehicle::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS || what & OUT_SHOCKS) {
        GetLog() << "\n---- Spring and Shock information\n\n";
        for (int axle = 0; axle < 2; axle++) {
            std::string axlePosition = (axle == 0) ? "Front" : "Rear ";
            for (int side = LEFT; side <= RIGHT; side++) {
                for (auto& forceTSDA :
                     m_axles[axle]->m_suspension->ReportSuspensionForce(static_cast<VehicleSide>(side))) {
                    GetLog() << axlePosition << " " << (side == LEFT ? "Left " : "Right") << " ";
                    GetLog() << forceTSDA.name << std::string(10 - std::max(0, (int)forceTSDA.name.size()), ' ')
                             << " Length: " << forceTSDA.length << " m, Force: " << forceTSDA.force << " N\n";
                }
            }
        }
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}
/// @} vehicle_models_vertiwheeler
}  // namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
