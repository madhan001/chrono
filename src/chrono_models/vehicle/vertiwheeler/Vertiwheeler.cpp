#include "chrono/ChConfig.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/vertiwheeler/Vertiwheeler.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace vertiwheeler {
/// @addtogroup vehicle_models_vertiwheeler
/// @{
// -----------------------------------------------------------------------------
Vertiwheeler::Vertiwheeler()
    : m_system(nullptr),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
    //   m_contactMethod(ChContactMethod::SMC),
      m_collsysType(ChCollisionSystem::Type::BULLET),  
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1.2),
      m_voltage_ratio(1),
      m_rolling_friction_coeff(0.05) {}

Vertiwheeler::Vertiwheeler(ChSystem* system)
    : m_system(system),
      m_vehicle(nullptr),
      m_contactMethod(ChContactMethod::NSC),
    //   m_contactMethod(ChContactMethod::SMC),
      m_collsysType(ChCollisionSystem::Type::BULLET),
      m_chassisCollisionType(CollisionType::NONE),
      m_fixed(false),
      m_tireType(TireModelType::RIGID),
      m_tire_step_size(-1),
      m_initFwdVel(0),
      m_initPos(ChCoordsys<>(ChVector3d(0, 0, 1), QUNIT)),
      m_initOmega({0, 0, 0, 0}),
      m_apply_drag(false),
      m_stall_torque(1.2),
      m_voltage_ratio(1) {}

Vertiwheeler::~Vertiwheeler() {
    delete m_vehicle;
}

// -----------------------------------------------------------------------------
void Vertiwheeler::SetAerodynamicDrag(double Cd, double area, double air_density) {
    m_Cd = Cd;
    m_area = area;
    m_air_density = air_density;

    m_apply_drag = true;
}

// -----------------------------------------------------------------------------
void Vertiwheeler::Initialize() {
    // Create and initialize the Vertiwheeler vehicle
    m_vehicle = m_system ? new Vertiwheeler_Vehicle(m_system, m_fixed, m_chassisCollisionType)
                         : new Vertiwheeler_Vehicle(m_fixed, m_contactMethod, m_chassisCollisionType);
    m_vehicle->SetCollisionSystemType(m_collsysType);
    m_vehicle->SetInitWheelAngVel(m_initOmega);
    m_vehicle->Initialize(m_initPos, m_initFwdVel);

    // If specified, enable aerodynamic drag
    if (m_apply_drag) {
        m_vehicle->GetChassis()->SetAerodynamicDrag(m_Cd, m_area, m_air_density);
    }

    // Create and initialize the powertrain system
    auto engine = chrono_types::make_shared<Vertiwheeler_EngineSimpleMap>("Engine");
    auto transmission = chrono_types::make_shared<Vertiwheeler_AutomaticTransmissionSimpleMap>("Transmission");
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    
    engine->m_voltage_ratio = m_voltage_ratio;
    engine->m_stall_torque = m_stall_torque;
    engine->m_supply_voltage = 13.9; // TITAN 550 14.4V
    engine->m_Kv_rating = 1200; // 21000 rpm/V


    std::cout << "Engine Initialized\n";
    std::cout << "Motor Voltage Ratio: " << engine->m_voltage_ratio << "\n";
    std::cout << "Motor Stall Torque: " << engine->m_stall_torque << "\n";
    std::cout << "Motor Kv: " << engine->m_Kv_rating << "\n";
    std::cout << "Motor Supply Voltage: " << engine->m_supply_voltage << "\n";
    std::cout << "Motor Max RPM: " << engine->GetMaxEngineSpeed() << "\n\n\n";

    m_vehicle->InitializePowertrain(powertrain);

    std::cout << "Powertrain Initialized\n\n\n";

    // Create the tires and set parameters depending on type.
    // bool use_mesh = (m_tireType == TireModelType::RIGID_MESH);
    bool use_mesh = false;

    switch (m_tireType) {
        case TireModelType::TMEASY: {
            auto tire_FL = chrono_types::make_shared<Vertiwheeler_TMeasyTire>("FL");
            auto tire_FR = chrono_types::make_shared<Vertiwheeler_TMeasyTire>("FR");
            auto tire_RL = chrono_types::make_shared<Vertiwheeler_TMeasyTire>("RL");
            auto tire_RR = chrono_types::make_shared<Vertiwheeler_TMeasyTire>("RR");

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::NONE);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::NONE);
            
            tire_FL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_FR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);

            tire_FL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_FR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RL->SetRollingResistanceCoefficient(m_rolling_friction_coeff);
            tire_RR->SetRollingResistanceCoefficient(m_rolling_friction_coeff);

            m_tire_mass = tire_FL->GetMass();
            break;
        }
        default: {  // Defaults to rigid tires
            auto tire_FL = chrono_types::make_shared<Vertiwheeler_RigidTire>("FL", use_mesh);
            auto tire_FR = chrono_types::make_shared<Vertiwheeler_RigidTire>("FR", use_mesh);
            auto tire_RL = chrono_types::make_shared<Vertiwheeler_RigidTire>("RL", use_mesh);
            auto tire_RR = chrono_types::make_shared<Vertiwheeler_RigidTire>("RR", use_mesh);

            m_vehicle->InitializeTire(tire_FL, m_vehicle->GetAxle(0)->m_wheels[LEFT], VisualizationType::PRIMITIVES);
            m_vehicle->InitializeTire(tire_FR, m_vehicle->GetAxle(0)->m_wheels[RIGHT], VisualizationType::PRIMITIVES);
            m_vehicle->InitializeTire(tire_RL, m_vehicle->GetAxle(1)->m_wheels[LEFT], VisualizationType::PRIMITIVES);
            m_vehicle->InitializeTire(tire_RR, m_vehicle->GetAxle(1)->m_wheels[RIGHT], VisualizationType::PRIMITIVES);
            m_tire_mass = tire_FL->GetMass();
            std::cout << "Tire mass: " << m_tire_mass << "\n";
            break;
        }
    }

    for (auto& axle : m_vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            if (m_tire_step_size > 0)
                wheel->GetTire()->SetStepsize(m_tire_step_size);
                std::cout << "Tire step size: " << m_tire_step_size << "\n";
        }
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    m_vehicle->InitializeInertiaProperties();
}

// -----------------------------------------------------------------------------
void Vertiwheeler::Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain) {
    m_vehicle->Synchronize(time, driver_inputs, terrain);
}

// -----------------------------------------------------------------------------
void Vertiwheeler::Advance(double step) {
    m_vehicle->Advance(step);
}
/// @} vehicle_models_vertiwheeler
}  // namespace vertiwheeler
}  // namespace vehicle
}  // namespace chrono
