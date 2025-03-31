#ifndef VERTIWHEELER_SOLID3LINK_AXLE_H
#define VERTIWHEELER_SOLID3LINK_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidThreeLinkAxle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

class CH_MODELS_API Vertiwheeler_Solid3LinkAxle : public ChSolidThreeLinkAxle {
  public:
    Vertiwheeler_Solid3LinkAxle(const std::string& name);
    ~Vertiwheeler_Solid3LinkAxle();

  protected:
    virtual const ChVector3d getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getTriangleMass() const override { return m_triangleMass; }
    virtual double getLinkMass() const override { return m_linkMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector3d getAxleTubeCOM() const override { return ChVector3d(0, 0, 0); }

    virtual const ChVector3d& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector3d& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector3d& getTriangleInertia() const override { return m_triangleInertia; }
    virtual const ChVector3d& getLinkInertia() const override { return m_linkInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }
    
    // virtual void AddVisualizationAssets(VisualizationType vis) override;

    // Helper function to add visualization link (cylinder), hehe cylinder ( ͡° ͜ʖ ͡°)
    void AddVisualizationLink(std::shared_ptr<ChBody> body,
                              const ChVector3d pt_1,
                              const ChVector3d pt_2,
                              const double radius,
                              const ChColor& color);

    // Helper function to add visualization point (cylinder)
    void AddVisualizationPoint(std::shared_ptr<ChBody> body,
                              const ChVector3d pt_1,
                              const double radius,
                              const ChColor& color);


  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_axleShaftInertia;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_triangleMass;
    static const double m_linkMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const ChVector3d m_axleTubeInertia;
    static const ChVector3d m_spindleInertia;
    static const ChVector3d m_triangleInertia;
    static const ChVector3d m_linkInertia;

    static const double m_springCoefficient1;
    static const double m_springCoefficient2;
    static const double m_springRestLength;
    static const double m_springDesignLength;
    static const double m_springMinLength;
    static const double m_springMaxLength;

    static const double m_damperCoefCompression;
    static const double m_damperDegresCompression;
    static const double m_damperDegresExpansion;
    static const double m_damperCoefExpansion;

    static const double m_twin_tire_dist;
};

/// @} vehicle_models_vertiwheeler

}  // namespace vertiwheeler
}  // end namespace vehicle
}  // end namespace chrono

#endif