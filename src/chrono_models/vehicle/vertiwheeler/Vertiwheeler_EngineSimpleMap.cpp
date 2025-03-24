// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Jayne Henry, Luning Fang, Marcel Offermans
// =============================================================================

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_EngineSimpleMap.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{
const double rpm2rads = CH_PI / 30;

Vertiwheeler_EngineSimpleMap::Vertiwheeler_EngineSimpleMap(const std::string& name)
    : ChEngineSimpleMap(name),
      m_voltage_ratio(1.0),
      m_Kv_rating(1500),        // Titan 550 21T is approximately 1500 KV
      m_supply_voltage(14.0),   // 14V nominal voltage
      m_stall_torque(1.2),      // Estimated stall torque at 14V
      m_motor_resistance(0.004) // From ESC specifications
{}

double Vertiwheeler_EngineSimpleMap::GetMaxEngineSpeed() {
    double max_rpm = m_Kv_rating * m_supply_voltage * m_voltage_ratio;
    // std::cout << "Max RPM: " << max_rpm << "\n";
    return max_rpm * rpm2rads;
}

void Vertiwheeler_EngineSimpleMap::SetEngineTorqueMaps(ChFunctionInterp& map0, ChFunctionInterp& mapF) {
    double max_rpm = m_Kv_rating * m_supply_voltage * m_voltage_ratio;
    double max_speed = max_rpm * rpm2rads;
 
    // mapF.AddPoint(0, m_stall_torque);      // stall torque //TODO
    // mapF.AddPoint(max_rpm * rpm2rads, 0);  // no load speed

    // N-m and rad/s
    map0.AddPoint(0, 10e-7);      // Stall torque at zero speed
    map0.AddPoint(max_rpm * rpm2rads, 0);  // TODO, currently a guess

        // Full throttle map (linear approximation for ESC-controlled BLDC)
    mapF.AddPoint(0, m_stall_torque);      // Stall torque at zero speed
    mapF.AddPoint(max_speed * 0.5, m_stall_torque * 0.7);  // Mid-point
    mapF.AddPoint(max_speed, 0);           // No load speed
    
    // // Zero throttle map (motor braking characteristics)
    // map0.AddPoint(0, 0);
    // map0.AddPoint(max_speed * 0.3, -0.1 * m_stall_torque);  // Light braking at low speed
    // map0.AddPoint(max_speed, -0.2 * m_stall_torque);        // Increased braking at max speed
}
/// @} vehicle_models_vertiwheeler
}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
