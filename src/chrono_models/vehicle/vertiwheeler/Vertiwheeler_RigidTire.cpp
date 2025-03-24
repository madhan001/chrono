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
// Vertiwheeler rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_RigidTire.h"

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

const double Vertiwheeler_RigidTire::m_radius = .117/2; // actual value
const double Vertiwheeler_RigidTire::m_width = .041; // actual value

const double Vertiwheeler_RigidTire::m_mass = .143; // 163g per tire and wheel, 20g for wheel, 143g for tire
const ChVector<> Vertiwheeler_RigidTire::m_inertia(.0008, 0.001, .0008);

const std::string Vertiwheeler_RigidTire::m_meshName = "Vertiwheeler_tire_POV_geom";
// const std::string Vertiwheeler_RigidTire::m_meshFile = "vertiwheeler/tire.obj";

const std::string Vertiwheeler_RigidTire::m_meshFile_left = "vertiwheeler/tire_L_scaled.obj";
const std::string Vertiwheeler_RigidTire::m_meshFile_right = "vertiwheeler/tire_R_scaled.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void Vertiwheeler_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f; //rubber on concrete
    // minfo.mu = 0.01f; //rubber on ice

    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
    // m_material = minfo.CreateMaterial(ChContactMethod::NSC);
    // GetLog()<< "BP : Vertiwheeler_RigidTire::CreateContactMaterial\n";

}

Vertiwheeler_RigidTire::Vertiwheeler_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Vertiwheeler_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Vertiwheeler_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}
/// @} vehicle_models_vertiwheeler
}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
