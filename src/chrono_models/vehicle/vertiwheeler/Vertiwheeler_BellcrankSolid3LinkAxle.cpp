#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler_BellcrankSolid3LinkAxle.h"

#include "chrono/utils/ChUtilsCreators.h"


namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------


const double mass_scale_factor = 0.01;       // 1/10 scale mass
const double dimension_scale_factor = 0.1;  // 1/10 scale dimensions
const double inertia_scale_factor = mass_scale_factor * dimension_scale_factor * dimension_scale_factor; // Typically mass * dimension^2
const double force_scale_factor = mass_scale_factor * dimension_scale_factor; // 1/100 scale for Newtons
const double spring_rate_scale_factor = force_scale_factor / dimension_scale_factor; // Force / dimension
const double damping_rate_scale_factor = force_scale_factor / dimension_scale_factor; // Force / velocity


const double Vertiwheeler_BellcrankSolid3LinkAxle::m_axleTubeMass = 11.25 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_spindleMass = 1.67 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_bellcrankMass = 0.68 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_knuckleMass = 4.04 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_draglinkMass = 0.29 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_triangleMass = 1.39 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_linkMass = 0.69 * mass_scale_factor * 0.1;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_tierodMass = 0.28 * mass_scale_factor * 0.1;
// const double common_mass = 0.03;
const double common_link_radius = 0.001;


const double Vertiwheeler_BellcrankSolid3LinkAxle::m_axleTubeRadius = 0.007933333333333333;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_spindleRadius = 0.016666666666666666;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_spindleWidth = 0.01;

const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_axleTubeInertia = ChVector<>(0.59, 0.23, 0.59) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_spindleInertia = ChVector<>(0.0075, 0.0134, 0.0075) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_bellcrankInertia = ChVector<>(0.0014, 0.008, 0.0083) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_knuckleInertia = ChVector<>(0.067, 0.11, 0.068) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_draglinkInertia = ChVector<>(0.008, 0.019, 0.026) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_triangleInertia = ChVector<>(0.0056, 0.0056, 0.0056) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_linkInertia = ChVector<>(0.0014, 0.0028, 0.0028) * inertia_scale_factor;
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::m_tierodInertia = ChVector<>(0.0014, 0.0028, 0.0029) * inertia_scale_factor;

// const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springDesignLength = 0.09; // 1/10 scale
// Spring Rate Conversion:
// 3.10 in/lb for both springs in parallel.
// for each spring: 3.10 in/lb * (1 lb / 4.448 N) * (39.37 in / 1 m) = 27.45 N/m
// 27.45 N/m * (1 m / 1000 mm) = 0.02745 N/mm

// last good values
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springCoefficient1 = 0.02745 * 10e4;  // Linear spring rate (N/mm)
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springCoefficient2 = 0.013725 * 10e4;  // Quadratic spring rate (N/mm^2) - a fraction of the original quadratic term
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springRestLength = 0.07; // 1/10 scale
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springMinLength = 0.04; // 1/10 scale
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springMaxLength = 0.08; // 1/10 scale

const double Vertiwheeler_BellcrankSolid3LinkAxle::m_damperCoefExpansion = 98727.9 * damping_rate_scale_factor;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_damperDegresExpansion = 2.0; // Reduced from 4.77954
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_damperCoefCompression = 52526.6 * damping_rate_scale_factor;
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_damperDegresCompression = 1.0; // Reduced from 3.0

const double Vertiwheeler_BellcrankSolid3LinkAxle::m_axleShaftInertia = 0.4 * inertia_scale_factor;

const double Vertiwheeler_BellcrankSolid3LinkAxle::m_twin_tire_dist = 0.0;

// new set of values, copied from rear susp
const double Vertiwheeler_BellcrankSolid3LinkAxle::m_springDesignLength = 0.09; // 1/10 scale
// ---------------------------------------------------------------------------------------
// MAN 5t spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class CH_MODELS_API Vertiwheeler_SpringForceFront : public ChLinkTSDA::ForceFunctor {
  public:
    Vertiwheeler_SpringForceFront(double spring_constant1, double spring_coefficient2, double min_length, double max_length);

    virtual double evaluate(double time,
                            double rest_length,
                            double length,
                            double vel,
                            const ChLinkTSDA& link) override;

  private:
    double m_spring_constant1;
    double m_spring_constant2;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;
};

Vertiwheeler_SpringForceFront::Vertiwheeler_SpringForceFront(double spring_constant1,
                                                 double spring_constant2,
                                                 double min_length,
                                                 double max_length)
    : m_spring_constant1(spring_constant1),
      m_spring_constant2(spring_constant2),
      m_min_length(min_length),
      m_max_length(max_length) {

    m_bump.AddPoint(0.0, 0.0);
    m_bump.AddPoint(2.0e-3 * dimension_scale_factor, 200.0 * force_scale_factor);
    m_bump.AddPoint(4.0e-3 * dimension_scale_factor, 400.0 * force_scale_factor);
    m_bump.AddPoint(6.0e-3 * dimension_scale_factor, 600.0 * force_scale_factor);
    m_bump.AddPoint(8.0e-3 * dimension_scale_factor, 800.0 * force_scale_factor);
    m_bump.AddPoint(10.0e-3 * dimension_scale_factor, 1000.0 * force_scale_factor);
    m_bump.AddPoint(20.0e-3 * dimension_scale_factor, 2500.0 * force_scale_factor);
    m_bump.AddPoint(30.0e-3 * dimension_scale_factor, 4500.0 * force_scale_factor);
    m_bump.AddPoint(40.0e-3 * dimension_scale_factor, 7500.0 * force_scale_factor);
    m_bump.AddPoint(50.0e-3 * dimension_scale_factor, 12500.0 * force_scale_factor);

}

double Vertiwheeler_SpringForceFront::evaluate(double time,
                                         double rest_length,
                                         double length,
                                         double vel,
                                         const ChLinkTSDA& link) {
    double force = 0;

    double defl_spring = rest_length - length;
    double defl_bump = 0.0;
    double defl_rebound = 0.0;

    if (length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if (length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = defl_spring * m_spring_constant1 + defl_spring * std::abs(defl_spring) * m_spring_constant2 +
            m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

Vertiwheeler_BellcrankSolid3LinkAxle::Vertiwheeler_BellcrankSolid3LinkAxle(const std::string& name)
    : ChSolidBellcrankThreeLinkAxle(name) {
    m_springForceCB = chrono_types::make_shared<Vertiwheeler_SpringForceFront>(m_springCoefficient1, m_springCoefficient2,
                                                                         m_springMinLength, m_springMaxLength);

    m_shockForceCB = chrono_types::make_shared<DegressiveDamperForce>(
        m_damperCoefCompression, m_damperDegresCompression, m_damperCoefExpansion, m_damperDegresExpansion);
    

}


// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Vertiwheeler_BellcrankSolid3LinkAxle::~Vertiwheeler_BellcrankSolid3LinkAxle() {}

// -----------------------------------------------------------------------------
// safe values : https://aistudio.google.com/prompts/1OEBo-UZDmkufYSdPrP0glcxCkpktAsDx
const ChVector<> Vertiwheeler_BellcrankSolid3LinkAxle::getLocation(PointId which) {
    // measurements scaled so that SPINDLE is at (0.0, 0.1, 0.0)
    // BASELINE VALUES
    switch (which) {
        case SPRING_A:    // (0.000, 0.590, 0.090)
            return ChVector<>(0.0000, 0.0571, 0.0087);

        case SPRING_C:    // (0.010, 0.552, 0.588)
            return ChVector<>(0.0010, 0.0535, 0.0569);

        case SHOCK_A:     // (0.246, 0.523, -0.125)
            return ChVector<>(0.0238, 0.0507, -0.0121);

        case SHOCK_C:     // (0.235, 0.562, 0.570)
            return ChVector<>(0.0228, 0.0544, 0.0552);

        case SPINDLE:     // (0.0, 1.033, 0.0)
            return ChVector<>(0.0000, 0.1000, 0.0000);

        case TRIANGLE_A:  // (0.125, 0.000, 0.260)
            return ChVector<>(0.0121, 0.0000, 0.0252);

        case TRIANGLE_C:  // (0.871, 0.420, 0.165)
            return ChVector<>(0.0845, 0.0406, 0.0160);

        case LINK_A:      // (-0.115, 0.490, -0.090)
            return ChVector<>(-0.0111, 0.0474, -0.0087);

        case LINK_C:      // (-1.138, 0.270, 0.115)
            return ChVector<>(-0.1102, 0.0261, 0.0111);

        case DRAGLINK_S:  // (0.741, -0.217, 0.089)
            return ChVector<>(0.0718, -0.0210, 0.0086);

        case BELLCRANK_A: // (-0.023, 0.000, 0.250)
            return ChVector<>(-0.0022, 0.0000, 0.0242);

        case BELLCRANK_D: // (0.045, 0.256, 0.153)
            return ChVector<>(0.0044, 0.0248, 0.0148);

        case BELLCRANK_T: // (-0.273, 0.042, 0.153)
            return ChVector<>(-0.0264, 0.0041, 0.0148);

        case KNUCKLE_L:   // (0.000, 0.853748866, -0.100)
            return ChVector<>(0.0000, 0.0826, -0.0097);

        case KNUCKLE_U:   // (0.000, 0.836251134,  0.100)
            return ChVector<>(0.0000, 0.0810,  0.0097);

        case KNUCKLE_T:   // (-0.236, 0.800, 0.153)
            return ChVector<>(-0.0228, 0.0774, 0.0148);

        case KNUCKLE_CM:  // (0.000, 0.937, 0.000)
            return ChVector<>(0.0000, 0.0907, 0.0000);

        default:
            return ChVector<>(0, 0, 0);

    }
}


// -----------------------------------------------------------------------------
// Visualization override
// -----------------------------------------------------------------------------

void Vertiwheeler_BellcrankSolid3LinkAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                         const ChVector<> pt_1,
                                                         const ChVector<> pt_2,
                                                         const double radius,
                                                         const ChColor& color) {
    // no need for ChVehicleGeometry::AddVisualizationLink                                            
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    // utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius); // new version : see, https://github.com/zzhou292/chrono/commit/bf1f664aa81e5f8747eac2e63ea345d0cb7e03f2#diff-5fc1d2d793c4feda8f34d207a492ddd076c8ee35dfc0cf054a9047b16e597fd0L69

    // ChVehicleGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    ChVehicleGeometry::AddVisualizationCylinder(body, pt_1, pt_2, radius);

}


void Vertiwheeler_BellcrankSolid3LinkAxle::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::PRIMITIVES){

        // visualize the axle tube
        AddVisualizationLink(m_axleTube, getLocation(SPINDLE), getLocation(SPINDLE) * -1, 0.002, ChColor(0.3f, 0.3f, 0.3f));
        // All points in getLocation(enum) are in the axleTube frame, so pass m_axleTube body 
        // visualize the link bodies
        AddVisualizationLink(m_axleTube, getLocation(LINK_A), getLocation(LINK_C), 0.003, ChColor(0.3f, 0.3f, 1.0f));
        AddVisualizationLink(m_axleTube, getLocation(LINK_A) * ChVector<>(1,-1,1), getLocation(LINK_C) * ChVector<>(1,-1,1), 0.003, ChColor(0.3f, 0.3f, 1.0f));

        //visualize the triangle body
        AddVisualizationLink(m_axleTube, getLocation(TRIANGLE_A), getLocation(TRIANGLE_C), 0.003, ChColor(0.3f, 0.3f, 1.0f));
        AddVisualizationLink(m_axleTube, getLocation(TRIANGLE_A) * ChVector<>(1,-1,1), getLocation(TRIANGLE_C) * ChVector<>(1,-1,1), 0.003, ChColor(0.3f, 0.3f, 1.0f));

        // visualize the draglink
        // visualize BELLCRANK_T - KNUCKLE_T
        AddVisualizationLink(m_axleTube, getLocation(BELLCRANK_T), getLocation(KNUCKLE_T) , 0.002, ChColor(0.6f, 0.3f, 0.3f));
        AddVisualizationLink(m_axleTube, getLocation(BELLCRANK_T) * ChVector<>(1,-1,1), getLocation(KNUCKLE_T) * ChVector<>(1,-1,1), 0.002, ChColor(0.6f, 0.3f, 0.3f));
        
        // visualize the knuckle kingpin
        AddVisualizationLink(m_axleTube, getLocation(KNUCKLE_L), getLocation(KNUCKLE_U), 0.002, ChColor(0.6f, 0.3f, 0.3f));
        AddVisualizationLink(m_axleTube, getLocation(KNUCKLE_L) * ChVector<>(1,-1,1), getLocation(KNUCKLE_U) * ChVector<>(1,-1,1), 0.002, ChColor(0.6f, 0.3f, 0.3f));

        // visualize the bellcrank 
        AddVisualizationLink(m_axleTube, getLocation(BELLCRANK_A), getLocation(BELLCRANK_A) + ChVector<>(0,0,0.004), 0.002, ChColor(0.6f, 0.3f, 0.3f));

        // ChVehicleGeometry::SphereShape(m_knuckle[LEFT],)
        // AddVisualizationLink(m_linkBody[LEFT], getLocation(LINK_A), getLocation(LINK_C), 0.005, ChColor(0.3f, 0.3f, 1.0f));
        // AddVisualizationPoint(m_axleTube, getLocation(BELLCRANK_T), 0.01, ChColor(0.6f, 0.3f, 0.3f));

        m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.006, 150, 15));
        m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.006, 150, 15));
        m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    }
}

/// @} vehicle_models_vertiwheeler

}  // namespace vertiwheeler
}  // end namespace vehicle
}  // end namespace chrono


