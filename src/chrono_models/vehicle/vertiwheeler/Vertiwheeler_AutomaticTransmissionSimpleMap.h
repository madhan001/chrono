#ifndef VWMEDIUM_AUTOMATICTRANSMISSIONSIMPLEMAP_H
#define VWMEDIUM_AUTOMATICTRANSMISSIONSIMPLEMAP_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"
#include "chrono_models/ChApiModels.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

/// Simple Vertiwheeler transmission subsystem.
class CH_MODELS_API Vertiwheeler_AutomaticTransmissionSimpleMap : public chrono::vehicle::ChAutomaticTransmissionSimpleMap {
  public:
    Vertiwheeler_AutomaticTransmissionSimpleMap(const std::string& name);

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) override;
};

/// @} vehicle_models_vertiwheeler

}  // end namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono

#endif
