#include "topdown_car.hpp"
USING_NS_CC;
USING_NS_CC_EXT;

TDTire::TDTire(cocos2d::Layer *layer, b2World * world)
{
	sprite_ = PhysicsSprite::create("tire.png");
	sprite_->setOpacity(100);
	sprite_->setTag(CAR_TAG);

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.userData = sprite_;
	body_ = world->CreateBody(&bodyDef);

	b2PolygonShape polygonShape;
	polygonShape.SetAsBox( sprite_->getContentSize().width / PTM_RATIO / 2,  sprite_->getContentSize().height /PTM_RATIO / 2);
	//polygonShape.SetAsBox( 5,  12.5f);
	b2Fixture* fixture = body_->CreateFixture(&polygonShape, DENSITY);//shape, density

	sprite_->setB2Body(body_);
	sprite_->setPTMRatio(PTM_RATIO);
	layer->addChild(sprite_);

	currentTraction_ = 1;
}

TDTire::~TDTire()
{
}

void TDTire::updateFriction()
{
	updateTraction();
	//lateral linear velocity
	auto impulse = body_->GetMass() * -getLateralVelocity();
	body_->ApplyLinearImpulse(impulse, body_->GetWorldCenter(), true);

	//forward linear velocity
	b2Vec2 currentForwardNormal = getForwardVelocity();
	float currentForwardSpeed = currentForwardNormal.Normalize();
	float dragForceMagnitude = DRAG_MANITITUDE * currentForwardSpeed;
	body_->ApplyForce(dragForceMagnitude * currentForwardNormal, body_->GetWorldCenter(), true);
}

b2Vec2 TDTire::getLateralVelocity() {
    b2Vec2 currentRightNormal = body_->GetWorldVector( b2Vec2(1,0) );
    return b2Dot( currentRightNormal, body_->GetLinearVelocity() ) * currentRightNormal;
}

b2Vec2 TDTire::getForwardVelocity() {
    b2Vec2 currentForwardNormal = body_->GetWorldVector( b2Vec2(0,1) );
    return b2Dot( currentForwardNormal, body_->GetLinearVelocity() ) * currentForwardNormal;
}

void TDTire::updateDrive(const Direction & direction, bool brake, bool accelerator)
{
	//find current speed in forward direction
	b2Vec2 currentForwardNormal = body_->GetWorldVector( b2Vec2(0,1) );
	float currentSpeed = b2Dot( getForwardVelocity(), currentForwardNormal);
	//CCLOG("current Speed = %f", currentSpeed);

	// Brake takes priority over accelerator
	if(brake) {
		//@todo Set a maximum brake impulse to allow skid
		auto impulse = body_->GetMass() * -getForwardVelocity();
		body_->ApplyLinearImpulse(impulse, body_->GetWorldCenter(), true);
	}
	else if(accelerator) {
		// find desired speed
		float desiredSpeed = 0;
		switch (direction) {
			case Direction::FORWARD: desiredSpeed = maxForwardSpeed_;  break;
			case Direction::BACKWARD: desiredSpeed = maxBackwardSpeed_; break;
			default: return;//do nothing
		}

		//apply necessary force
		float force = 0;
		if ( desiredSpeed > currentSpeed )
			force = maxDriveForce_;
		else if ( desiredSpeed < currentSpeed )
			force = -maxDriveForce_;
		else
			return;
		body_->ApplyForce(currentTraction_ * force * currentForwardNormal, body_->GetWorldCenter(), true);
	}
}

TDCar::TDCar(cocos2d::Layer *layer, b2World *world)
{
	const auto visibleSize = Director::getInstance()->getVisibleSize();
	const auto origin = Director::getInstance()->getVisibleOrigin();

	sprite_ = PhysicsSprite::create("car.png");
	sprite_->setOpacity(100);
	sprite_->setTag(CAR_TAG);
	const int width = sprite_->getContentSize().width, height = sprite_->getContentSize().height;

	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.userData = sprite_;
	bodyDef.position = {visibleSize.width / PTM_RATIO, visibleSize.height / PTM_RATIO};
	body_ = world->CreateBody(&bodyDef);
	body_->SetAngularDamping(3);

	// box2d wants anti-clockwise winding, coco2d wants clockwise winding
//	b2Vec2 vertices[] = {{-width / 2.0f, -height / 2.0f}, {-width / 2.0f, height / 2.0f}, {width / 2.0f, height / 2.0f}, {width / 2.0f, -height / 2.0f}};
//	reverse(begin(vertices), end(vertices)); // TODO: for box2d

	b2PolygonShape polygonShape;
	polygonShape.SetAsBox( (sprite_->getContentSize().width / PTM_RATIO / 2),  (sprite_->getContentSize().height / PTM_RATIO / 2));
	b2Fixture* fixture = body_->CreateFixture(&polygonShape, CAR_BODY_DENSITY);//shape, density

	sprite_->setB2Body(body_);
	sprite_->setPTMRatio(PTM_RATIO);
	sprite_->setPosition({origin.x + visibleSize.width / 2, origin.y + visibleSize.height / 2});
	layer->addChild(sprite_);

	//prepare common joint parameters
	b2RevoluteJointDef jointDef;
	jointDef.bodyA = body_;
	jointDef.enableLimit = true;
	jointDef.lowerAngle = 0;
	jointDef.upperAngle = 0;
	jointDef.localAnchorB.SetZero();//center of tire

	//back left tire
	TDTire* blTire = new TDTire(layer, world);
	blTire->setCharacteristics(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED, BACK_TIRE_MAX_DRIVE_FORCE, BACK_TIRE_MAX_LATERAL_IMPULSE);
	jointDef.bodyB = blTire->body();
	jointDef.localAnchorA.Set( -TIRE_ANCHOR_X / PTM_RATIO, -TIRE_ANCHOR_Y / PTM_RATIO );
	world->CreateJoint( &jointDef );
	tires_.push_back(blTire);

	//back right tire
	auto brTire = new TDTire(layer, world);
	brTire->setCharacteristics(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED, BACK_TIRE_MAX_DRIVE_FORCE, BACK_TIRE_MAX_LATERAL_IMPULSE);
	jointDef.bodyB = brTire->body();
	jointDef.localAnchorA.Set( TIRE_ANCHOR_X / PTM_RATIO, -TIRE_ANCHOR_Y / PTM_RATIO );
	world->CreateJoint( &jointDef );
	tires_.push_back(brTire);

	//front left tire
	auto flTire = new TDTire(layer, world);
	flTire->setCharacteristics(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED, FRONT_TIRE_MAX_DRIVE_FORCE, FRONT_TIRE_MAX_LATERAL_IMPULSE);
	jointDef.bodyB = flTire->body();
	jointDef.localAnchorA.Set( -TIRE_ANCHOR_X / PTM_RATIO, TIRE_ANCHOR_Y / PTM_RATIO );
	flJoint_ = (b2RevoluteJoint*)world->CreateJoint( &jointDef );
	tires_.push_back(flTire);

	//front right tire
	auto frTire = new TDTire(layer, world);
	frTire->setCharacteristics(MAX_FORWARD_SPEED, MAX_BACKWARD_SPEED, FRONT_TIRE_MAX_DRIVE_FORCE, FRONT_TIRE_MAX_LATERAL_IMPULSE);
	jointDef.bodyB = frTire->body();
	jointDef.localAnchorA.Set( TIRE_ANCHOR_X / PTM_RATIO, TIRE_ANCHOR_Y / PTM_RATIO );
	frJoint_ = (b2RevoluteJoint*)world->CreateJoint( &jointDef );
	tires_.push_back(frTire);

	//body_->SetTransform({origin.x + visibleSize.width / 2, origin.y + visibleSize.height / 2}, 0);
}

TDCar::~TDCar() {
	for (int i = 0; i < tires_.size(); i++)
		delete tires_[i];
}

void TDCar::update(const Direction & direction, const Steering & steering, bool brake, bool accelerator) {
	for (int i = 0; i < tires_.size(); i++)
		tires_[i]->updateFriction();

	for (int i = 0; i < tires_.size(); i++)
		tires_[i]->updateDrive(direction, brake, accelerator);

	//control steering
	float lockAngle = FRONT_WHEEL_LOCK_ANGLE * DEGTORAD;
	float turnSpeedPerSec = 160 * DEGTORAD;//from lock to lock in 0.5 sec
	float turnPerTimeStep = turnSpeedPerSec / 60.0f;
	float desiredAngle = 0;
	switch (steering) {
	case Steering::LEFT:  desiredAngle = lockAngle;  break;
	case Steering::RIGHT: desiredAngle = -lockAngle; break;
	default: break;//nothing
	}
	float angleNow = flJoint_->GetJointAngle();
	float angleToTurn = desiredAngle - angleNow;
	angleToTurn = b2Clamp( angleToTurn, -turnPerTimeStep, turnPerTimeStep );
	float newAngle = angleNow + angleToTurn;
	flJoint_->SetLimits(newAngle, newAngle);
	frJoint_->SetLimits(newAngle, newAngle);
}

TopdownCar::TopdownCar(cocos2d::Layer * layer, b2World *world)
{
	//m_world->SetDestructionListener(&m_destructionListener);
	m_car = new TDCar(layer, world);
}

TopdownCar::~TopdownCar()
{
	//delete m_tire;
	delete m_car;
}

void TopdownCar::update(const Direction & direction, const Steering & steering, bool brake, bool accelerator)
{
	m_car->update(direction, steering, brake, accelerator);
}
