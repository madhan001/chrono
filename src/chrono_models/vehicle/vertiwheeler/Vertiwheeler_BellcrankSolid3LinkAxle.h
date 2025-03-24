// =============================================================================
//
// v4w (front) steered and driven solid three link axle.
//
// =============================================================================

#ifndef VWMEDIUM_BELLCRANK_SOLID3LINK_AXLE_H
#define VWMEDIUM_BELLCRANK_SOLID3LINK_AXLE_H

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.h"
#include "chrono/utils/ChUtilsCreators.h"
// #include "chrono/utils/ChBodyGeometry.h"

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"


#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace vertiwheeler {

/// @addtogroup vehicle_models_vertiwheeler
/// @{

class CH_MODELS_API Vertiwheeler_BellcrankSolid3LinkAxle : public ChSolidBellcrankThreeLinkAxle {
  public:
    Vertiwheeler_BellcrankSolid3LinkAxle(const std::string& name);
    ~Vertiwheeler_BellcrankSolid3LinkAxle();

  protected:
    virtual const ChVector<> getLocation(PointId which) override;

    virtual double getAxleTubeMass() const override { return m_axleTubeMass; }
    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual double getKnuckleMass() const override { return m_knuckleMass; }
    virtual double getBellcrankMass() const override { return m_knuckleMass; }
    virtual double getDraglinkMass() const override { return m_draglinkMass; }
    virtual double getTriangleMass() const override { return m_triangleMass; }
    virtual double getLinkMass() const override { return m_linkMass; }
    virtual double getTierodMass() const override { return m_tierodMass; }

    virtual double getAxleTubeRadius() const override { return m_axleTubeRadius; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    // virtual double getBellCrankRadius() const override { return m_bellCrankRadius; }

    virtual double getSpindleWidth() const override { return m_spindleWidth; }

    virtual const ChVector<> getAxleTubeCOM() const override { return ChVector<>(0, 0, 0); }

    virtual const ChVector<>& getAxleTubeInertia() const override { return m_axleTubeInertia; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }
    virtual const ChVector<>& getKnuckleInertia() const override { return m_knuckleInertia; }
    virtual const ChVector<>& getBellcrankInertia() const override { return m_bellcrankInertia; }
    virtual const ChVector<>& getDraglinkInertia() const override { return m_draglinkInertia; }
    virtual const ChVector<>& getTriangleInertia() const override { return m_triangleInertia; }
    virtual const ChVector<>& getLinkInertia() const override { return m_linkInertia; }
    virtual const ChVector<>& getTierodInertia() const override { return m_tierodInertia; }

    virtual double getAxleInertia() const override { return m_axleShaftInertia; }

    virtual double getSpringRestLength() const override { return m_springRestLength; }
    /// Return the functor object for spring force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getSpringForceFunctor() const override { return m_springForceCB; }
    /// Return the functor object for shock force.
    virtual std::shared_ptr<ChLinkTSDA::ForceFunctor> getShockForceFunctor() const override { return m_shockForceCB; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;

    // Helper function to add visualization link (cylinder)
    void AddVisualizationLink(std::shared_ptr<ChBody> body,
                              const ChVector<> pt_1,
                              const ChVector<> pt_2,
                              const double radius,
                              const ChColor& color);

    // Helper function to add visualization point (cylinder)
    void AddVisualizationPoint(std::shared_ptr<ChBody> body,
                              const ChVector<> pt_1,
                              const double radius,
                              const ChColor& color);
  private:
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_springForceCB;
    std::shared_ptr<ChLinkTSDA::ForceFunctor> m_shockForceCB;

    static const double m_bellcrankVisRadius;

    static const double m_axleShaftInertia;

    static const double m_axleTubeMass;
    static const double m_spindleMass;
    static const double m_knuckleMass;
    static const double m_bellcrankMass;
    static const double m_draglinkMass;
    static const double m_triangleMass;
    static const double m_linkMass;
    static const double m_tierodMass;

    static const double m_axleTubeRadius;
    static const double m_spindleRadius;
    static const double m_spindleWidth;

    static const double m_bellCrankRadius;

    static const ChVector<> m_axleTubeInertia;
    static const ChVector<> m_spindleInertia;
    static const ChVector<> m_knuckleInertia;
    static const ChVector<> m_bellcrankInertia;
    static const ChVector<> m_draglinkInertia;
    static const ChVector<> m_triangleInertia;
    static const ChVector<> m_linkInertia;
    static const ChVector<> m_tierodInertia;

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

    // virtual const ChVector<> getLocation(PointId which);
    
    // static void AddVisualizationLink(std::shared_ptr<ChBody> body,
                                    //  const ChVector<> pt_1,
                                    //  const ChVector<> pt_2,
                                    //  double radius,
                                    //  const ChColor& color);

};

/// @} vehicle_models_vertiwheeler

}  // namespace vertiwheeler
}  // end namespace vehicle
}  // end namespace chrono

#endif


