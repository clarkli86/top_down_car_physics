#ifndef __GAME_SCENE_H__
#define __GAME_SCENE_H__

#include "cocos2d.h"
#include "topdown_car.hpp"
#include "GLES-Render.h"
#include "contact_listener.hpp"
class GearBox;
class Pedal;

class GameScene : public cocos2d::Layer
{
public:
    // there's no 'id' in cpp, so we recommend returning the class instance pointer
    static cocos2d::Scene* createScene();

    // Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
    virtual bool init();

    // a selector callback
    void menuCloseCallback(cocos2d::Ref* pSender);

    // implement the "static create()" method manually
    CREATE_FUNC(GameScene);
private:
    bool onTouchBegan(cocos2d::Touch *touch, cocos2d::Event *event);
    void onTouchEnded(cocos2d::Touch *touch, cocos2d::Event *event);
    bool onContactBegin(cocos2d::PhysicsContact &contact);
    void updatePhysics(float dt);

    // Create car and direction control. This must be called after physicsWorld_ has been initialised
    void createControls();
    // Init the physics world and debug draw
    void initPhysics();

    // Enable debug draw of Box2D
    void draw(cocos2d::Renderer *renderer, const cocos2d::Mat4 &transform, uint32_t flags) override;
#if CC_ENABLE_BOX2D_INTEGRATION
protected:
    cocos2d::Mat4 _modelViewMV;
    void onDraw(const cocos2d::Mat4 &transform, uint32_t flags);
    cocos2d::CustomCommand _customCommand;
#endif
    GLESDebugDraw m_debugDraw;

    b2World *world_ = nullptr;

    cocos2d::Sprite *l = nullptr;
    cocos2d::Sprite *r = nullptr;

    TopdownCar * car_;
    GearBox * gearbox_;
    Pedal * brake_;
    Pedal * accelerator_;
    Steering steering_ = Steering::NONE;
    cocos2d::TMXTiledMap * tileMap_;
    cocos2d::TMXLayer * layer_;

    ContactListener contactListener_;
};

#endif // __GAME_SCENE_H__
