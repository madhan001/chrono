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
// FEDA Rack-Pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/feda/FEDA_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double FEDA_RackPinion::m_steeringLinkMass = 9.072;
const ChVector3d FEDA_RackPinion::m_steeringLinkInertia(1, 1, 1);
const double FEDA_RackPinion::m_steeringLinkCOM = 0.0;
const double FEDA_RackPinion::m_steeringLinkRadius = 0.03;
const double FEDA_RackPinion::m_steeringLinkLength = 0.896;
const double FEDA_RackPinion::m_pinionRadius = 0.10;
const double FEDA_RackPinion::m_maxAngle = 50 * (CH_DEG_TO_RAD);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_RackPinion::FEDA_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono