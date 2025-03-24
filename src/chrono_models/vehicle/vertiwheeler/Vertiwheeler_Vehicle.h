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
// Base class for the Vertiwheeler vehicle models
//
// =============================================================================

#ifndef VWMEDIUM_VEHICLE_H
#define VWMEDIUM_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_BrakeSimple.h"
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Chassis.h"
// #include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_DoubleWishbone.h"

// solid axle suspension
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Solid3LinkAxle.h"
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_BellcrankSolid3LinkAxle.h"
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_RotaryArm.h"

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Driveline4WD.h"
// #include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_PitmanArm.h"
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Wheel.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

/// Vertiwheeler vehicle system.
class CH_MODELS_API Vertiwheeler_Vehicle : public ChWheeledVehicle {
  public:
    Vertiwheeler_Vehicle(const bool fixed, ChContactMethod contact_method, CollisionType chassis_collision_type);

    Vertiwheeler_Vehicle(ChSystem* system, const bool fixed, CollisionType chassis_collision_type);

    ~Vertiwheeler_Vehicle();

    virtual unsigned int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 0.34; }
    virtual double GetMinTurningRadius() const override { return 0.6; }
    virtual double GetMaxSteeringAngle() const override { return 45.0 * CH_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

    // double GetSpringForce(int axle, VehicleSide side) const;
    // double GetSpringLength(int axle, VehicleSide side) const;
    // double GetSpringDeformation(int axle, VehicleSide side) const;

    // double GetShockForce(int axle, VehicleSide side) const;
    // double GetShockLength(int axle, VehicleSide side) const;
    // double GetShockVelocity(int axle, VehicleSide side) const;

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed, CollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

/// @} vehicle_models_vertiwheeler

}  // namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono

#endif
