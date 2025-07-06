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
// Authors: Tong Xu
// =============================================================================
//
// Front and Rear FEDA suspension subsystems (reduced double A-arm)
// Based on FEDA full double wishbone parameters, adapted for rack-pinion steering.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_DoubleWishboneReduced.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables - adapted from FEDA full double wishbone
// -----------------------------------------------------------------------------

// Use FEDA upright + spindle mass (from original FEDA values)
const double FEDA_DoubleWishboneReducedFront::m_uprightMass = 76.27;
const double FEDA_DoubleWishboneReducedFront::m_spindleMass = 13.08;

const double FEDA_DoubleWishboneReducedFront::m_spindleRadius = 0.10;
const double FEDA_DoubleWishboneReducedFront::m_spindleWidth = 0.06;
const double FEDA_DoubleWishboneReducedFront::m_uprightRadius = 0.04;

// Use FEDA inertia values
const ChVector3d FEDA_DoubleWishboneReducedFront::m_spindleInertia(5.32e-4, 5.52e-4, 5.32e-4);
const ChVector3d FEDA_DoubleWishboneReducedFront::m_uprightInertia(0.1656, 0.1934, 0.04367);

const double FEDA_DoubleWishboneReducedFront::m_axleInertia = 0.4;

// Use FEDA spring parameters with simplified damping for reduced suspension
const double FEDA_DoubleWishboneReducedFront::m_springCoefficient = 76000;
const double FEDA_DoubleWishboneReducedFront::m_dampingCoefficient = 3000; // Simplified damping
const double FEDA_DoubleWishboneReducedFront::m_springRestLength = 0.60208;

// -----------------------------------------------------------------------------

const double FEDA_DoubleWishboneReducedRear::m_uprightMass = 76.27;
const double FEDA_DoubleWishboneReducedRear::m_spindleMass = 13.08;

const double FEDA_DoubleWishboneReducedRear::m_spindleRadius = 0.10;
const double FEDA_DoubleWishboneReducedRear::m_spindleWidth = 0.06;
const double FEDA_DoubleWishboneReducedRear::m_uprightRadius = 0.04;

const ChVector3d FEDA_DoubleWishboneReducedRear::m_spindleInertia(5.32e-4, 5.52e-4, 5.32e-4);
const ChVector3d FEDA_DoubleWishboneReducedRear::m_uprightInertia(0.1656, 0.1934, 0.04367);

const double FEDA_DoubleWishboneReducedRear::m_axleInertia = 0.4;

const double FEDA_DoubleWishboneReducedRear::m_springCoefficient = 76000;
const double FEDA_DoubleWishboneReducedRear::m_dampingCoefficient = 3000; // Simplified damping
const double FEDA_DoubleWishboneReducedRear::m_springRestLength = 0.60208;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
FEDA_DoubleWishboneReducedFront::FEDA_DoubleWishboneReducedFront(const std::string& name, int rideHeightMode, int damperMode)
    : ChDoubleWishboneReduced(name), m_ride_height_mode(rideHeightMode), m_damper_mode(damperMode) {
    // Use simple linear spring-damper for reduced suspension
    m_shockForceCB = chrono_types::make_shared<LinearSpringDamperForce>(m_springCoefficient, m_dampingCoefficient);
}

FEDA_DoubleWishboneReducedRear::FEDA_DoubleWishboneReducedRear(const std::string& name, int rideHeightMode, int damperMode)
    : ChDoubleWishboneReduced(name), m_ride_height_mode(rideHeightMode), m_damper_mode(damperMode) {
    // Use simple linear spring-damper for reduced suspension
    m_shockForceCB = chrono_types::make_shared<LinearSpringDamperForce>(m_springCoefficient, m_dampingCoefficient);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
FEDA_DoubleWishboneReducedFront::~FEDA_DoubleWishboneReducedFront() {}

FEDA_DoubleWishboneReducedRear::~FEDA_DoubleWishboneReducedRear() {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// Using FEDA geometry but with modified TIEROD_C for rack-pinion steering
// -----------------------------------------------------------------------------

const ChVector3d FEDA_DoubleWishboneReducedFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return ChVector3d(0.0, 0.97663, 0);
        case UPRIGHT:
            return ChVector3d(0, 0.87, 0);  // Simplified - single upright position
        case UCA_F:
            return ChVector3d(0.0478, 0.2324, 0.3469);
        case UCA_B:
            return ChVector3d(-0.3215, 0.2324, 0.3469);
        case UCA_U:
            return ChVector3d(-0.01759, 0.6744, 0.30589);
        case LCA_F:
            return ChVector3d(0.16781, 0.2245, -0.08);
        case LCA_B:
            return ChVector3d(-0.45219, 0.22245, -0.119);
        case LCA_U:
            return ChVector3d(0.00789, 0.80719, -0.13904);
        case SHOCK_C:
            return ChVector3d(0.09397, 0.493925, 0.46209);
        case SHOCK_U:
            return ChVector3d(0.09397, 0.65153, -0.119);  // Combined spring-shock mount
        case TIEROD_C:
            return ChVector3d(-0.24078, 0.379095, 0.04);
        case TIEROD_U:
            return ChVector3d(-0.207, 0.82618, 0);
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d FEDA_DoubleWishboneReducedRear::getLocation(PointId which) {
    switch (which) {
        // Keep ALL suspension geometry identical to full double wishbone
        case SPINDLE:
            return ChVector3d(0.0, 0.97663, 0);
        case UPRIGHT:
            return ChVector3d(0, 0.87, 0);
        case UCA_F:
            return ChVector3d(0.0478, 0.2324, 0.3469);
        case UCA_B:
            return ChVector3d(-0.3215, 0.2324, 0.3469);
        case UCA_U:
            return ChVector3d(-0.01759, 0.6744, 0.30589);
        case LCA_F:
            return ChVector3d(0.16781, 0.2245, -0.08);
        case LCA_B:
            return ChVector3d(-0.45219, 0.22245, -0.119);
        case LCA_U:
            return ChVector3d(0.00789, 0.80719, -0.13904);
        case SHOCK_C:
            return ChVector3d(-0.09397, 0.493925, 0.46209);  // Note: negative X for rear
        case SHOCK_U:
            return ChVector3d(-0.09397, 0.65153, -0.119);
        
        // Rear axle - typically no steering connections for rack-pinion
        case TIEROD_C:
            return ChVector3d(0.0, 0.0, 0.04);  // Centered (no rear steering)
        case TIEROD_U:
            return ChVector3d(0.0, 0.82618, 0);  // Centered connection
            
        default:
            return ChVector3d(0, 0, 0);
    }
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono


