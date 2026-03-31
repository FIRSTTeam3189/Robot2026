#pragma once

#include <memory>
#include <vector>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>

#include "simulation/box2d/box2d.h"


namespace simulation {
struct FieldMap {
public:
    struct ObstacleDef {
        b2BodyDef bodyDef;

        /*
            The reason we make a unique_ptr is that we cannot store
            a direct b2Shape as b2Shape is an abstract class.
        */
        std::unique_ptr<b2Shape> shape;
    };

    std::vector<ObstacleDef> obstacles;
    
    /**
     * Returns a static `Bodydef` that all
     * obstacles must use to define their shape
     * 
     * @return b2BodyDef
     */
    b2BodyDef CreateObstacleDef() {
        b2BodyDef obstacleBodyDef;
        obstacleBodyDef.type = b2BodyType::b2_staticBody;

        return obstacleBodyDef;
    }

    /**
     * Adds a convex polygon from the passed
     * vertices to the physics world
     * 
     * @param vertices The vertices of the convex shape, ordered in counterclockwise direction
     * @param fieldAbsolutePosition The center point of the convex shape.
     */
    void AddConvexShape(const std::vector<b2Vec2>& vertices, frc::Pose2d fieldAbsolutePosition) {
        b2BodyDef convexObstacle = CreateObstacleDef();
        convexObstacle.position.Set(
            fieldAbsolutePosition.X().value(),
            fieldAbsolutePosition.Y().value()
        );

        auto convexShape = std::make_unique<b2PolygonShape>();
        convexShape->Set(vertices.data(), (int32)vertices.size());
        
        obstacles.emplace_back(
            ObstacleDef{convexObstacle, std::move(convexShape)}
        );
    }

    /**
     * Adds a collidable line segment to the
     * physics world
     * 
     * @param startPoint The start point of the line segment
     * @param endPoint The end point of the line segment
     */
    void AddBorderLine(frc::Translation2d startPoint, frc::Translation2d endPoint) {
        b2BodyDef lineObstacle = CreateObstacleDef();

        b2Vec2 middlePosition = b2Vec2(
            (startPoint.X().value() + endPoint.X().value()) / 2,
            (startPoint.Y().value() + endPoint.Y().value()) / 2
        );

        lineObstacle.position.Set(0,0);

        // shape definition
        auto lineShape = std::make_unique<b2EdgeShape>();
        lineShape->SetTwoSided(
            b2Vec2(startPoint.X().value(), startPoint.Y().value()),
            b2Vec2(endPoint.X().value(), endPoint.Y().value())
        );

        obstacles.emplace_back(
            ObstacleDef{lineObstacle, std::move(lineShape)}
        );
    }
    /**
     * Adds a collidable rectangle shape to the
     * physics world
     * 
     * @param width The width of the rectangle in meters
     * @param height The height of the rectangle in meters
     * @param fieldAbsolutePosition The center position of the rectangle
     */
    void AddRectangularObstacle(double width, double height, frc::Pose2d fieldAbsolutePosition) {
        b2BodyDef rectangleObstacle = CreateObstacleDef();
        
        rectangleObstacle.position.Set(
            fieldAbsolutePosition.X().value(), 
            fieldAbsolutePosition.Y().value()
        );
        
        auto rectangleShape = std::make_unique<b2PolygonShape>();
        rectangleShape->SetAsBox(width / 2, height / 2);

        obstacles.emplace_back(
            ObstacleDef{rectangleObstacle, std::move(rectangleShape)}
        );
    };
};
}
