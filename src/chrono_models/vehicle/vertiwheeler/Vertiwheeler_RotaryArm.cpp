#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_RotaryArm.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Vertiwheeler_RotaryArm::m_pitmanArmMass = 0.02;

const double Vertiwheeler_RotaryArm::m_pitmanArmRadius = 0.002;

const double Vertiwheeler_RotaryArm::m_maxAngle = 45.0 * (CH_PI / 180);

const ChVector3d Vertiwheeler_RotaryArm::m_pitmanArmInertiaMoments(0.00638, 0.00756, 0.00150);
const ChVector3d Vertiwheeler_RotaryArm::m_pitmanArmInertiaProducts(0.0, 0.0, 0.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Vertiwheeler_RotaryArm::Vertiwheeler_RotaryArm(const std::string& name) : ChRotaryArm(name) {}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() and getDirection() virtual methods.
// -----------------------------------------------------------------------------


const ChVector3d Vertiwheeler_RotaryArm::getLocation(PointId which) {
    switch (which) {
        // oversteers when doing a left turn
        case ARM_L:
            return ChVector3d(0.0718, -0.0210, 0.0086);

        case ARM_C:
            return ChVector3d(0.17, -0.0028, 0.0166); // should be closer to the bellcrank point in chassis frame for it to work
        default:
            return ChVector3d(0, 0, 0);
    }
}

const ChVector3d Vertiwheeler_RotaryArm::getDirection(DirectionId which) {
    switch (which) {
        case REV_AXIS:
            return ChVector3d(0, 0, -1);
        default:
            return ChVector3d(0, 0, 1);
    }
}
/// @} vehicle_models_vertiwheeler
}  // namespace vertiwheeler
}  // end namespace vehicle
}  // end namespace chrono