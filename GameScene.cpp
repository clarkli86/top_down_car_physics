#include "GameScene.h"
#include "gear_box.hpp"
#include "pedal.hpp"
#include "defs.h"

USING_NS_CC;
USING_NS_CC_EXT;

Scene* GameScene::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();

    // 'layer' is an autorelease object
    auto layer = GameScene::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool GameScene::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }

    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

    /////////////////////////////
    // 2. add a menu item with "X" image, which is clicked to quit the program
    //    you may modify it.

    // add a "close" icon to exit the progress. it's an autorelease object
    auto closeItem = MenuItemImage::create(
                                           "CloseNormal.png",
                                           "CloseSelected.png",
                                           CC_CALLBACK_1(GameScene::menuCloseCallback, this));

	closeItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width/2 ,
                                origin.y + closeItem->getContentSize().height/2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, NULL);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);

    /////////////////////////////
    // 3. add your codes below...
    // Background
    tileMap_ = TMXTiledMap::create("side_parking.tmx");
    layer_ = tileMap_->layerNamed("background");
    // Create the edge
    auto edgeBody = PhysicsBody::createEdgeBox(layer_->getContentSize(), PHYSICSSHAPE_MATERIAL_DEFAULT, 3);
    edgeBody->setContactTestBitmask(CAR);
    edgeBody->setCollisionBitmask(CAR);
    edgeBody->setCategoryBitmask(OBSTACLE);
    layer_->setPhysicsBody(edgeBody);
	layer_->setPosition(origin.x + layer_->getContentSize().width / 2, origin.y + layer_->getContentSize().height / 2);
	//this->addChild(layer_, TILED_MAP_Z_ORDER);

	// Create gear box
	gearbox_ = GearBox::create();
	gearbox_->setOpacity(CONTROLS_OPACITY);
	gearbox_->setPosition({origin.x + visibleSize.width - gearbox_->getContentSize().width / 2, origin.y + visibleSize.height / 2});
	this->addChild(gearbox_, CONTROLS_Z_ORDER);

	// Create the brake
	brake_ = Pedal::create("brake.png");
	brake_->setOpacity(CONTROLS_OPACITY);
	brake_->setPosition({origin.x + visibleSize.width / 2, origin.y + brake_->getContentSize().height / 2});
	this->addChild(brake_, CONTROLS_Z_ORDER);

	accelerator_ = Pedal::create("accelerator.png");
	accelerator_->setOpacity(CONTROLS_OPACITY);
	accelerator_->setPosition({origin.x + visibleSize.width - accelerator_->getContentSize().width / 2, origin.y + + accelerator_->getContentSize().height / 2});
	this->addChild(accelerator_, CONTROLS_Z_ORDER);

	initPhysics();

	createControls();

    this->schedule(schedule_selector(GameScene::updatePhysics));

    return true;
}

void GameScene::menuCloseCallback(Ref* pSender)
{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WP8) || (CC_TARGET_PLATFORM == CC_PLATFORM_WINRT)
	MessageBox("You pressed the close button. Windows Store Apps do not implement a close button.","Alert");
    return;
#endif

    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}

void GameScene::initPhysics()
{
	// Create Box2d physics world
	world_ = new b2World({0, 0});

	// Do we want to let bodies sleep?
	world_->SetAllowSleeping(true);
	world_->SetContinuousPhysics(true);

	world_->SetDebugDraw(&m_debugDraw);
	uint32 flags = 0;
	flags += b2Draw::e_shapeBit;
	flags += b2Draw::e_jointBit;
	//flags += b2Draw::e_aabbBit;
	flags += b2Draw::e_centerOfMassBit;
	m_debugDraw.SetFlags(flags);

	world_->SetContactListener(&contactListener_);

	// Define the ground body.
	b2BodyDef rockBodyDef;
	rockBodyDef.type = b2_dynamicBody;

	auto sprite = PhysicsSprite::create("rock.png");
	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b2Body* rockBody = world_->CreateBody(&rockBodyDef);
	b2PolygonShape rockShape;
	rockShape.SetAsBox(sprite->getContentSize().width / PTM_RATIO / 2, sprite->getContentSize().height / PTM_RATIO / 2);

	b2FixtureDef rockFixture;
	rockFixture.shape = &rockShape;
	rockFixture.friction = 1;
	rockFixture.density = 10;
	rockBody->CreateFixture(&rockFixture);

	sprite->setB2Body(rockBody);
	sprite->setPTMRatio(PTM_RATIO);
	sprite->setPosition({500, 500});
	addChild(sprite);
}

void GameScene::createControls()
{
	car_ = new TopdownCar(this, world_);

	l = Sprite::create("left.png"), r = Sprite::create("right.png");
	l->setPosition(100, 100);
	r->setPosition(400, 100);
	addChild(l, CONTROLS_Z_ORDER, LEFT_SPRITE_TAG);
	addChild(r, CONTROLS_Z_ORDER, RIGHT_SPRITE_TAG);

    // Create touch listener
    auto touchListener = EventListenerTouchOneByOne::create();
    touchListener->onTouchBegan = CC_CALLBACK_2(GameScene::onTouchBegan, this);
    touchListener->onTouchEnded = CC_CALLBACK_2(GameScene::onTouchEnded, this);
    Director::getInstance()->getEventDispatcher()->addEventListenerWithSceneGraphPriority(touchListener, this);
//
//    auto contactListener = EventListenerPhysicsContact::create();
//    contactListener->onContactBegin = CC_CALLBACK_1(HelloWorld::onContactBegin, this);
//    Director::getInstance()->getEventDispatcher()->addEventListenerWithSceneGraphPriority(contactListener, this);
}

bool GameScene::onTouchBegan(cocos2d::Touch *touch, cocos2d::Event *event)
{
	auto location =	convertTouchToNodeSpace(touch);
	// getBoudingBox() returns coordinates in parent's coordinate system to match location
	if(l->getBoundingBox().containsPoint(location)) steering_ = Steering::LEFT;
	else if(r->getBoundingBox().containsPoint(location)) steering_ = Steering::RIGHT;
	return true;
}

void GameScene::onTouchEnded(cocos2d::Touch *touch, cocos2d::Event *event)
{
	auto location =	convertTouchToNodeSpace(touch);
	// getBoudingBox() returns coordinates in parent's coordinate system to match location
	if(l->getBoundingBox().containsPoint(location) || r->getBoundingBox().containsPoint(location))
		steering_ = Steering::NONE;
}

void GameScene::updatePhysics(float dt)
{
	if(!car_) return;

	// Whether to apply brake
	const bool brake = brake_->pressed();
	const bool accelerator = accelerator_->pressed();
	const Direction direction =
		gearbox_->gear() == GearBox::Gear::FORWARD ? Direction::FORWARD : Direction::BACKWARD;

	car_->update(direction, steering_, brake, accelerator);

    const int32 velocityIterations = 8;   //how strongly to correct velocity
    const int32 positionIterations = 3;   //how strongly to correct position

    world_->Step( dt, velocityIterations, positionIterations);
}

bool GameScene::onContactBegin(cocos2d::PhysicsContact &contact)
{
	PhysicsBody *a = contact.getShapeA()->getBody();
	PhysicsBody *b = contact.getShapeB()->getBody();
	CCLOG("a->category = %08x, a->contact = %08x, b->category = %08x, b->contact = %08x",
			a->getCategoryBitmask(), a->getContactTestBitmask(), b->getCategoryBitmask(), b->getContactTestBitmask());

	return true;
}

void GameScene::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
    //
    // IMPORTANT:
    // This is only for debug purposes
    // It is recommend to disable it
    //
    Layer::draw(renderer, transform, flags);

    _customCommand.init(_globalZOrder);
    _customCommand.func = CC_CALLBACK_0(GameScene::onDraw, this, transform, flags);
    renderer->addCommand(&_customCommand);
}

#if CC_ENABLE_BOX2D_INTEGRATION
void GameScene::onDraw(const Mat4 &transform, uint32_t flags)
{
	Director* director = Director::getInstance();
	CCASSERT(nullptr != director, "Director is null when seting matrix stack");
	director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
	director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, transform);

	GL::enableVertexAttribs( cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION );
	world_->DrawDebugData();
	CHECK_GL_ERROR_DEBUG();

	director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}
#endif
