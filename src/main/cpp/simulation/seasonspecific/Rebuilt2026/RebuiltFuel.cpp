#include "simulation/seasonspecific/Rebuilt2026/RebuiltFuel.h"


namespace simulation::pieces {
    RebuiltFuel::RebuiltFuel(b2World* m_physicsWorld, frc::Pose3d initialPose)
    : GamePiece(
        GetPieceInfo(),
        m_physicsWorld, 
        initialPose
    )
    {
        
    };

    PieceInfo RebuiltFuel::GetPieceInfo() {
        auto fuelShape = std::make_unique<b2CircleShape>();
        b2BodyDef fuelBodyDef;
        fuelBodyDef.linearDamping = 0.3f;
        fuelBodyDef.angularDamping = 5.0f;
        fuelBodyDef.allowSleep = true;
        fuelBodyDef.awake = false;

        fuelShape->m_radius = units::meter_t{7.5_cm}.value();
        fuelBodyDef.type = b2BodyType::b2_dynamicBody;

        return PieceInfo{
            .shape              = std::move(fuelShape),
            .bodyDef            = fuelBodyDef,
            .type               = "Fuel",
            .height             = units::meter_t{15_cm},
            .mass               = 0.5_lb,
            .restitution        = 0.8
        };
    }


}