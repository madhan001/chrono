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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// MAN 5t (front) steered and driven solid three link axle.
//
// =============================================================================

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Solid3LinkAxle.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double mass_scale_factor = 0.01;       // 1/10 scale mass
const double dimension_scale_factor = 0.1;  // 1/10 scale dimensions
const double inertia_scale_factor = mass_scale_factor * dimension_scale_factor * dimension_scale_factor; // Typically mass * dimension^2
const double force_scale_factor = mass_scale_factor * dimension_scale_factor; // 1/100 scale for Newtons
const double spring_rate_scale_factor = force_scale_factor / dimension_scale_factor; // Force / dimension
const double damping_rate_scale_factor = force_scale_factor / dimension_scale_factor; // Force / velocity

const double Vertiwheeler_Solid3LinkAxle::m_axleTubeMass = 709 * mass_scale_factor * 0.1;
const double Vertiwheeler_Solid3LinkAxle::m_spindleMass = 14.705 * 4.1 * mass_scale_factor * 0.1;
const double Vertiwheeler_Solid3LinkAxle::m_triangleMass = 50.0 * mass_scale_factor * 0.1;
const double Vertiwheeler_Solid3LinkAxle::m_linkMass = 25.0 * mass_scale_factor * 0.1;


const double Vertiwheeler_Solid3LinkAxle::m_axleTubeRadius = 0.0476 * dimension_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_spindleRadius = 0.10 * dimension_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_spindleWidth = 0.06 * dimension_scale_factor;

const ChVector<> Vertiwheeler_Solid3LinkAxle::m_axleTubeInertia = ChVector<>(329.00, 16.46, 330.00) * inertia_scale_factor;
const ChVector<> Vertiwheeler_Solid3LinkAxle::m_spindleInertia = ChVector<>(0.04117 * 6.56, 0.07352 * 6.56, 0.04117 * 6.56) * inertia_scale_factor;
const ChVector<> Vertiwheeler_Solid3LinkAxle::m_triangleInertia = ChVector<>(0.2, 0.2, 0.2) * inertia_scale_factor;
const ChVector<> Vertiwheeler_Solid3LinkAxle::m_linkInertia = ChVector<>(0.05, 0.1, 0.1) * inertia_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_springDesignLength = 0.09; // 1/10 scale, 80mm
const double Vertiwheeler_Solid3LinkAxle::m_springCoefficient1 = 0.02745 * 100;  // Linear spring rate (N/mm)
const double Vertiwheeler_Solid3LinkAxle::m_springCoefficient2 = 0.013725 * 100;  // Quadratic spring rate (N/mm^2) - a fraction of the original quadratic term
const double Vertiwheeler_Solid3LinkAxle::m_springRestLength = 0.06; // 1/10 scale
const double Vertiwheeler_Solid3LinkAxle::m_springMinLength = 0.04; // 1/10 scale
const double Vertiwheeler_Solid3LinkAxle::m_springMaxLength = 0.07; // 1/10 scale

// Attempt 0 : Good enough
const double Vertiwheeler_Solid3LinkAxle::m_damperCoefExpansion = 98727.9 * damping_rate_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_damperDegresExpansion = 2.0 * 0.7; // Reduced from 4.77954, last good : 1.0
const double Vertiwheeler_Solid3LinkAxle::m_damperCoefCompression = 52526.6 * damping_rate_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_damperDegresCompression = 1.0 * 1; // Reduced from 3.0, last good : 1.0


// // Attempt 1 : bounces : underdamped

// const double Vertiwheeler_Solid3LinkAxle::m_damperCoefExpansion = 45000.0 * damping_rate_scale_factor;
// const double Vertiwheeler_Solid3LinkAxle::m_damperDegresExpansion = 1.0;  // Linear damping
// const double Vertiwheeler_Solid3LinkAxle::m_damperCoefCompression = 25000.0 * damping_rate_scale_factor;
// const double Vertiwheeler_Solid3LinkAxle::m_damperDegresCompression = 1.0;  // Linear damping

// Attempt 2 : soggy : overdamped : stuck in a pit

// const double Vertiwheeler_Solid3LinkAxle::m_damperCoefExpansion = 150000.0 * damping_rate_scale_factor;
// const double Vertiwheeler_Solid3LinkAxle::m_damperDegresExpansion = 1.0;  // Linear damping
// const double Vertiwheeler_Solid3LinkAxle::m_damperCoefCompression = 75000.0 * damping_rate_scale_factor;
// const double Vertiwheeler_Solid3LinkAxle::m_damperDegresCompression = 1.0;  // Linear damping

const double Vertiwheeler_Solid3LinkAxle::m_axleShaftInertia = 0.4 * inertia_scale_factor;
const double Vertiwheeler_Solid3LinkAxle::m_twin_tire_dist = 0.0;




// ---------------------------------------------------------------------------------------
// 1/10 scale car spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class CH_MODELS_API Vertiwheeler_SpringForceRear : public ChLinkTSDA::ForceFunctor {
  public:
    Vertiwheeler_SpringForceRear(double spring_constant1, double spring_constant2, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant1;
    double m_spring_constant2;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

Vertiwheeler_SpringForceRear::Vertiwheeler_SpringForceRear(double spring_constant1,
                                                   double spring_constant2,
                                                   double min_length,
                                                   double max_length)
    : m_spring_constant1(spring_constant1),
      m_spring_constant2(spring_constant2),
      m_min_length(min_length),
      m_max_length(max_length) {
    // Adjusted for 1/10 scale RC car - less aggressive bump stops
    // Scaled down from original
    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3 * dimension_scale_factor, 200.0 * force_scale_factor );
    m_bump.AddPoint(4.0e-3 * dimension_scale_factor, 400.0 * force_scale_factor );
    m_bump.AddPoint(6.0e-3 * dimension_scale_factor, 600.0 * force_scale_factor );
    m_bump.AddPoint(8.0e-3 * dimension_scale_factor, 800.0 * force_scale_factor );
    m_bump.AddPoint(10.0e-3 * dimension_scale_factor, 1000.0 * force_scale_factor );
    m_bump.AddPoint(20.0e-3 * dimension_scale_factor, 2500.0 * force_scale_factor );
    m_bump.AddPoint(30.0e-3 * dimension_scale_factor, 4500.0 * force_scale_factor );
    m_bump.AddPoint(40.0e-3 * dimension_scale_factor, 7500.0 * force_scale_factor );
    m_bump.AddPoint(50.0e-3 * dimension_scale_factor, 12500.0 * force_scale_factor );

}

double Vertiwheeler_SpringForceRear::evaluate(double time,
                                        double rest_length,
                                        double length,
                                        double vel,
                                        const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
        // std::cout<<"Bump"<<std::endl;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
        // std::cout<<"Rebound"<<std::endl;
    }

    force = (defl_spring * m_spring_constant1) + (defl_spring * std::abs(defl_spring) * m_spring_constant2) +
            (m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound));
    
    return force;
}

// Force callbacks for the spring and shock (damper) force elements

Vertiwheeler_Solid3LinkAxle::Vertiwheeler_Solid3LinkAxle(const std::string& name) : ChSolidThreeLinkAxle(name) {
    m_springForceCB = chrono_types::make_shared<Vertiwheeler_SpringForceRear>(m_springCoefficient1, m_springCoefficient2,
                                                                        m_springMinLength, m_springMaxLength);
    
    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
    
    // double m_springCoefficient = 200; // 50 : lower end, 250 upper end, 95 : correct value for trx4 rear susp?
    // double m_springPreload = 0.015; // 1 cm preload
    // m_springForceCB = chrono_types::make_shared<LinearSpringForce>(m_springCoefficient, m_springPreload);

    
    // double m_dampingCoefficient = 300; // 50 : lower end, 250 upper end
    // m_shockForceCB = chrono_types::make_shared<LinearDamperForce>(m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Vertiwheeler_Solid3LinkAxle::~Vertiwheeler_Solid3LinkAxle() {}

// 
// placeholder, made-up values

// const ChVector<> Vertiwheeler_Solid3LinkAxle::getLocation(PointId which) {
//     // measurements scaled so that SPINDLE is at (0, 0.1, 0)
//     switch (which) {
//         case SPRING_A:
//             return ChVector<>(0.0, 0.063285, 0.0086956);
//         case SPRING_C:
//             return ChVector<>(0.0, 0.0565217, 0.0565217);
//         case SHOCK_A:
//             return ChVector<>(-0.0237681, 0.0664734, -0.0120772);
//         case SHOCK_C:
//             return ChVector<>(-0.0241545, 0.05430, 0.0550724);
//         case SPINDLE:
//             return ChVector<>(0.0, 0.1, 0.0);
//         case TRIANGLE_A:
//             return ChVector<>(0.0, 0.0, 0.0251207);
//         case TRIANGLE_C:
//             return ChVector<>(-0.0736231, 0.0405797, 0.0096618);
//         case LINK_A:
//             return ChVector<>(0.0111111, 0.0664734, -0.0086956);
//         case LINK_C:
//             return ChVector<>(0.1099516, 0.0386473, 0.0096618);
//         default:
//             return ChVector<>(0, 0, 0);
//     }
// }

// const ChVector<> Vertiwheeler_Solid3LinkAxle::getLocation(PointId which) {
//     double tsda_chassis_z_offset = 0.0;

//     switch (which) {
//         case SPRING_A:
//             return ChVector<>(0.01, 0.055, 0.01);  // "Spring" -> "Location Axle"
//         case SPRING_C:
//             return ChVector<>(0.0, 0.04, 0.09 + tsda_chassis_z_offset);  // "Spring" -> "Location Chassis"
//         case SHOCK_A:
//             return ChVector<>(0.01, 0.06, 0.01);  // "Shock" -> "Location Axle"
//         case SHOCK_C:
//             return ChVector<>(0.00, 0.05, 0.11 + tsda_chassis_z_offset);  // "Shock" -> "Location Chassis"
//         case SPINDLE:
//             return ChVector<>(0.0, 0.1, -0.1);  // "Spindle" -> "COM" y-component, scaled by 1/2 for each side
//         case TRIANGLE_A:
//             // return ChVector<>(0.0015, 0.0, -0.025);
//             return ChVector<>(0.0015, 0.0, 0.019);  // "Triangular Link" -> "Location Axle"
//         case TRIANGLE_C:
//             return ChVector<>(0.09, 0.025, 0.03 + tsda_chassis_z_offset);  // "Triangular Link" -> "Location Chassis"
//         case LINK_A:
//             return ChVector<>(0.0, 0.06, 0.01);  // "Longitudinal Link" -> "Location Axle"
//         case LINK_C: // needs to be attached to the chassis
//             return ChVector<>(-1 * 0.08, 0.03, 0.05 + tsda_chassis_z_offset);  // "Longitudinal Link" -> "Location Chassis"
//         default:
//             return ChVector<>(0, 0, 0);
//     }
// }


const ChVector<> Vertiwheeler_Solid3LinkAxle::getLocation(PointId which) {

    // Derivative of above
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.01, 0.055, 0.005);  // "Spring" -> "Location Axle"
        case SPRING_C:
            return ChVector<>(0.0, 0.04, 0.08);  // "Spring" -> "Location Chassis"
        case SHOCK_A: 
            return ChVector<>(0.01, 0.07, -0.01);  // "Shock" -> "Location Axle"
        case SHOCK_C:
            return ChVector<>(0.00, 0.05, 0.11);  // "Shock" -> "Location Chassis"
        case SPINDLE:
            return ChVector<>(0.0, 0.1, 0.0);  // "Spindle" -> "COM" y-component, scaled by 1/2 for each side
        case TRIANGLE_A:
            return ChVector<>(0.0015, 0.0, 0.0);  // "Triangular Link" -> "Location Axle"
        case TRIANGLE_C:
            return ChVector<>(0.10, 0.026, 0.03);  // "Triangular Link" -> "Location Chassis"
        case LINK_A:
            return ChVector<>(0.0, 0.06, 0);  // "Longitudinal Link" -> "Location Axle"
        case LINK_C: // needs to be attached to the chassis
            return ChVector<>(0.1, 0.03, 0.05);  // "Longitudinal Link" -> "Location Chassis"
        default:
            return ChVector<>(0, 0, 0);
    }   

}

// -----------------------------------------------------------------------------
// Visualization override
// -----------------------------------------------------------------------------

void Vertiwheeler_Solid3LinkAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                         const ChVector<> pt_1,
                                                         const ChVector<> pt_2,
                                                         const double radius,
                                                         const ChColor& color) {
    // no need for ChVehicleGeometry::AddVisualizationLink                                            
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    // utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius); // new version : see, https://github.com/zzhou292/chrono/commit/bf1f664aa81e5f8747eac2e63ea345d0cb7e03f2#diff-5fc1d2d793c4feda8f34d207a492ddd076c8ee35dfc0cf054a9047b16e597fd0L69

    // ChVehicleGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    ChVehicleGeometry::AddVisualizationCylinder(body, pt_1, pt_2, radius);

}
/// @} vehicle_models_vertiwheeler
}  // namespace vertiwheeler
}  // end namespace vehicle
}  // end namespace chrono