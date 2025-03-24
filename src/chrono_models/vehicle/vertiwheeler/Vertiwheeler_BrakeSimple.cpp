#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
/// @addtogroup vehicle_models_vertiwheeler
/// @{
const double Vertiwheeler_BrakeSimple::m_maxtorque = 1.2; //TODO

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Vertiwheeler_BrakeSimple::Vertiwheeler_BrakeSimple(const std::string& name) : chrono::vehicle::ChBrakeSimple(name) {}
/// @} vehicle_models_vertiwheeler

}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
