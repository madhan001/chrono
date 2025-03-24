#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_AutomaticTransmissionSimpleMap.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{
Vertiwheeler_AutomaticTransmissionSimpleMap::Vertiwheeler_AutomaticTransmissionSimpleMap(const std::string& name)
    : ChAutomaticTransmissionSimpleMap(name) {}

void Vertiwheeler_AutomaticTransmissionSimpleMap::SetGearRatios(std::vector<double>& fwd, double& rev) {
    rev = -1.0 / 3; // wrong value here and vehicle starts moving back when no acceleration is applied
    fwd.push_back(1.0 / 64); // gear ratio 
}

void Vertiwheeler_AutomaticTransmissionSimpleMap::SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) {
    // never shifts
}
/// @} vehicle_models_vertiwheeler

}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
