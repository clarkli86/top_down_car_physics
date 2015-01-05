#include "gear_box.hpp"
USING_NS_CC;

GearBox* GearBox::create()
{
	GearBox* pSprite = new GearBox();

    if (pSprite->initWithFile("gear_box_forward.png"))
    {
        pSprite->autorelease();

        pSprite->initOptions();

        pSprite->addEvents();

        return pSprite;
    }

    CC_SAFE_DELETE(pSprite);
    return NULL;
}

void GearBox::initOptions()
{
    // do things here like setTag(), setPosition(), any custom logic.
}

void GearBox::addEvents()
{
    auto listener = cocos2d::EventListenerTouchOneByOne::create();
    listener->setSwallowTouches(true);

    listener->onTouchBegan = [&](cocos2d::Touch* touch, cocos2d::Event* event)
    {
        cocos2d::Vec2 p = touch->getLocation();
        cocos2d::Rect rect = this->getBoundingBox();

        if(rect.containsPoint(p))
        {
            return true; // to indicate that we have consumed it.
        }

        return false; // we did not consume this event, pass thru.
    };

    listener->onTouchEnded = [=](cocos2d::Touch* touch, cocos2d::Event* event)
    {
    	GearBox::touchEvent(touch);
    };

    cocos2d::Director::getInstance()->getEventDispatcher()->addEventListenerWithFixedPriority(listener, 30);
}

void GearBox::touchEvent(cocos2d::Touch* touch)
{
	const char * texture = nullptr;
    if(gear_ == Gear::FORWARD) {
    	gear_ = Gear::BACKWARD;
    	texture = "gear_box_backward.png";
    }
    else {
    	gear_ = Gear::FORWARD;
    	texture = "gear_box_forward.png";
    }
    // Update picture
    this->setTexture(texture);
}
