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
// Authors: Rainer Gericke
// =============================================================================
//
// U401 TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/man/MAN_5t_RigidTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double MAN_5t_RigidTire::m_radius = 0.629;
const double MAN_5t_RigidTire::m_width = 0.385;

const double MAN_5t_RigidTire::m_mass = 104.0;
const ChVector3d MAN_5t_RigidTire::m_inertia(17.8651, 31.6623, 17.8651);

const std::string MAN_5t_RigidTire::m_meshFile_left = "MAN_Kat1/meshes/MAN_tire.obj";
const std::string MAN_5t_RigidTire::m_meshFile_right = "MAN_Kat1/meshes/MAN_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_RigidTire::MAN_5t_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
    if (use_mesh) {
        SetMeshFilename(GetDataFile("MAN_Kat1/meshes/MAN_tire.obj"), 0.005);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;      
    minfo.cr = 0.1f;      // Same restitution
    minfo.Y = 2e7f;       // Same Young's modulus
    m_material = minfo.CreateMaterial(contact_method);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void MAN_5t_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void MAN_5t_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
