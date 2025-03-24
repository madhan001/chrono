// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// Simple engine model for the Vertiwheeler vehicle.
// - based on torque-speed engine maps
// - both power and torque limited
//
// =============================================================================

#ifndef VWMEDIUM_ENGINESIMPLEMAP_H
#define VWMEDIUM_ENGINESIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChEngineSimpleMap.h"
#include "chrono_models/ChApiModels.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

/// Simple Vertiwheeler powertrain subsystem (based on engine speed-torque maps).
class CH_MODELS_API Vertiwheeler_EngineSimpleMap : public ChEngineSimpleMap {
  public:
    Vertiwheeler_EngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    double GetMaxEngineSpeed() override;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunctionInterp::AddPoint() function.
    void SetEngineTorqueMaps(ChFunctionInterp& map0,  ///< [out] engine map at zero throttle
                             ChFunctionInterp& mapF   ///< [out] engine map at full throttle
                             ) override;

  private:
    double m_voltage_ratio;
    double m_Kv_rating;
    double m_supply_voltage;
    double m_stall_torque;
    double m_motor_resistance;  // Motor internal resistance

    friend class CH_MODELS_API Vertiwheeler;
};

/// @} vehicle_models_vertiwheeler

}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono

#endif
