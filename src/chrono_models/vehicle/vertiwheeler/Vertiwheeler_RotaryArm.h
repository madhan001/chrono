#ifndef VWMEDIUM_ROTARY_ARM_H
#define VWMEDIUM_ROTARY_ARM_H

#include "chrono_vehicle/wheeled_vehicle/steering/ChRotaryArm.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

/// RotaryArm steering subsystem for the uaz vehicle.
class CH_MODELS_API Vertiwheeler_RotaryArm : public ChRotaryArm {
  public:
    Vertiwheeler_RotaryArm(const std::string& name);
    ~Vertiwheeler_RotaryArm() {}

    virtual double getPitmanArmMass() const override { return m_pitmanArmMass; }

    virtual double getPitmanArmRadius() const override { return m_pitmanArmRadius; }

    virtual const ChVector3d& getPitmanArmInertiaMoments() const override { return m_pitmanArmInertiaMoments; }
    virtual const ChVector3d& getPitmanArmInertiaProducts() const override { return m_pitmanArmInertiaProducts; }

    virtual double getMaxAngle() const override { return m_maxAngle; }

    virtual const ChVector3d getLocation(PointId which) override;
    virtual const ChVector3d getDirection(DirectionId which) override;

  private:
    static const double m_pitmanArmMass;

    static const double m_pitmanArmRadius;

    static const double m_maxAngle;

    static const ChVector3d m_pitmanArmInertiaMoments;
    static const ChVector3d m_pitmanArmInertiaProducts;
};

/// @} vehicle_models_vertiwheeler

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif