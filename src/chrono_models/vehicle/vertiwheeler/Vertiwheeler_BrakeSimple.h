#ifndef VWMEDIUM_BRAKESIMPLE_H
#define VWMEDIUM_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"


namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

/// Simple Vertiwheeler brake subsystem (torque applied directly to the spindle joint).
class CH_MODELS_API Vertiwheeler_BrakeSimple : public chrono::vehicle::ChBrakeSimple {
  public:
    Vertiwheeler_BrakeSimple(const std::string& name);
    virtual ~Vertiwheeler_BrakeSimple() {}

    virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

  private:
    static const double m_maxtorque;
};

/// @} vehicle_models_vertiwheeler

}  // namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono

#endif
