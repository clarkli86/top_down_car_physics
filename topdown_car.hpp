/*
* Author: Chris Campbell - www.iforce2d.net
* Modified: Clark Li <clark.li86@gmail.com>
*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef TOPDOWN_CAR_HPP
#define TOPDOWN_CAR_HPP
#include <cocos2d.h>
#include <vector>
#include <set>
#include "extensions/cocos-ext.h"
#include "Box2D/Box2D.h"
#include "defs.h"
#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

#define FRONT_WHEEL_DISTANCE 100
#define BACK_WHEEL_DISTANCE 120
#define GAP_WHEEL_CAR_BODY 40 // gap between wheel and car body outside
#define DENSITY 0.01
#define CAR_BODY_DENSITY 0.001
#define VELOCITY_LIMIT 100
#define BACK_TIRE_MAX_DRIVE_FORCE 30
#define FRONT_TIRE_MAX_DRIVE_FORCE 30
#define MAX_BACKWARD_SPEED -40
#define MAX_FORWARD_SPEED 250
#define LINEAR_DAMPING 1 // Critical damping. Oscillation not wanted
#define ANGULAR_DAMPING 1
#define DRAG_MANITITUDE -2
#define LATERAL_DRAG_MANITITUDE -200
#define FRONT_WHEEL_LOCK_ANGLE 35 // in degree
#define FRONT_TIRE_MAX_LATERAL_IMPULSE 7.5
#define BACK_TIRE_MAX_LATERAL_IMPULSE 8.5
#define BRAKE_MANITUDE -100000
#define TIRE_ANCHOR_X 96
#define TIRE_ANCHOR_Y 130
enum class Steering {
	NONE  = 0,
    LEFT  = 1,
    RIGHT = 2
};

enum class Direction {
	FORWARD = 0,
	BACKWARD = 1
};

//types of fixture user data
enum CollisionMask {
    CAR = 0x1,
    OBSTACLE = 0x2
};

enum class Zorder {
	CAR = 0x2,
	TIRE = 0x1
};

class TDTire {
public:
    TDTire(cocos2d::Layer *layer, b2World * world);
    ~TDTire();

    void setCharacteristics(float maxForwardSpeed, float maxBackwardSpeed, float maxDriveForce, float maxLateralImpulse)
    {
    	maxForwardSpeed_ = maxForwardSpeed;
        maxBackwardSpeed_ = maxBackwardSpeed;
        maxDriveForce_ = maxDriveForce;
        maxLateralImpulse_ = maxLateralImpulse;
    }
    void updateFriction();

    void updateDrive(const Direction & direction, bool brake, bool accelerator);

    cocos2d::Size size() { return sprite_->getContentSize(); }

    b2Body * body() { return body_; }
    cocos2d::extension::PhysicsSprite * sprite() { return sprite_; }
private:
    void updateTraction() { currentTraction_ = 1; }

    b2Vec2 getLateralVelocity();

    b2Vec2 getForwardVelocity();

	cocos2d::Vec2 lateralNormal();

	cocos2d::Vec2 forwardNormal();

    b2Body *body_;
    cocos2d::extension::PhysicsSprite * sprite_;

    float maxForwardSpeed_;
    float maxBackwardSpeed_;
    float maxDriveForce_;
    float maxLateralImpulse_;
    float currentTraction_;
};

class TDCar {
public:
    TDCar(cocos2d::Layer *layer, b2World *world);

    ~TDCar();

    void update(const Direction & direction, const Steering & steering, bool brake, bool accelerator);
private:
    b2Body *body_;
	std::vector<TDTire*> tires_;
	cocos2d::extension::PhysicsSprite *sprite_;
	b2RevoluteJoint *flJoint_;
	b2RevoluteJoint *frJoint_;
};

//
//class MyDestructionListener :  public b2DestructionListener
//{
//    void SayGoodbye(b2Fixture* fixture)
//    {
//        if ( FixtureUserData* fud = (FixtureUserData*)fixture->GetUserData() )
//            delete fud;
//    }
//
//    //(unused but must implement all pure virtual functions)
//    void SayGoodbye(b2Joint* joint) {}
//};

class TopdownCar
{
public:
	TopdownCar(cocos2d::Layer * layer, b2World *world);

    ~TopdownCar();

    void brake();

    void accelerate();

    /// @param direction Forward or backward
    /// @param steering Left, Right or None
    /// @param brake Whether brake is pressed
    /// @param accelerator Whether accelerator is pressed
    void update(const Direction & direction, const Steering & steering, bool brake, bool accelerator);
private:
    Direction direction_;
    //MyDestructionListener m_destructionListener;
    b2Body * m_groundBody;
    cocos2d::extension::PhysicsSprite * ground_;
    //TDTire* m_tire;
    TDCar* m_car;

};

#endif // TOPDOWN_CAR_HPP
