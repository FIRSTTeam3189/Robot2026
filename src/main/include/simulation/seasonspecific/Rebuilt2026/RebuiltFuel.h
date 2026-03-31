#pragma once

#include "simulation/GamePiece.h"

namespace simulation::pieces {
    class RebuiltFuel : public GamePiece {
    public:
        RebuiltFuel(b2World* m_physicsWorld, frc::Pose3d initialPose);
    
        static PieceInfo GetPieceInfo();

    private:
        
    };
}