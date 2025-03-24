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
// Authors: Radu Serban, Justin Madsen, Jayne Henry
// =============================================================================
//
// Vertiwheeler wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"


#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_Wheel.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254; 
static const double lb2kg = 0.453592;

const double Vertiwheeler_Wheel::m_mass = .020; //20g per wheel (whole mass is 163g for whole tire and wheel)
const ChVector3d Vertiwheeler_Wheel::m_inertia(.00007, 0.000136, .00007);

const double Vertiwheeler_Wheel::m_radius = .053/2;
const double Vertiwheeler_Wheel::m_width = .03; 

const std::string Vertiwheeler_WheelLeft::m_meshName = "wheel_L_POV_geom";
// const std::string Vertiwheeler_WheelLeft::m_meshFile = "vertiwheeler/wheel_L.obj";
const std::string Vertiwheeler_WheelLeft::m_meshFile = "vertiwheeler/wheel_L.obj";

const std::string Vertiwheeler_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string Vertiwheeler_WheelRight::m_meshFile = "vertiwheeler/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Vertiwheeler_Wheel::Vertiwheeler_Wheel(const std::string& name) : ChWheel(name) {}

Vertiwheeler_WheelLeft::Vertiwheeler_WheelLeft(const std::string& name) : Vertiwheeler_Wheel(name) {}

Vertiwheeler_WheelRight::Vertiwheeler_WheelRight(const std::string& name) : Vertiwheeler_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vertiwheeler_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        m_trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_trimesh_shape->SetMutable(false);
        m_spindle->AddVisualShape(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void Vertiwheeler_Wheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_spindle, m_trimesh_shape);
    ChWheel::RemoveVisualizationAssets();
}
/// @} vehicle_models_vertiwheeler
}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
