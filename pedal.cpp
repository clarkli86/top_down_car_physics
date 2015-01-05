#include "pedal.hpp"
USING_NS_CC;

Pedal* Pedal::create(const char * image)
{
	Pedal* pSprite = new Pedal();

    if (pSprite->initWithFile(image))
    {
        pSprite->autorelease();

        pSprite->initOptions();

        pSprite->addEvents();

        return pSprite;
    }

    CC_SAFE_DELETE(pSprite);
    return NULL;
}

void Pedal::initOptions()
{
    // do things here like setTag(), setPosition(), any custom logic.
}

void Pedal::addEvents()
{
    auto listener = cocos2d::EventListenerTouchOneByOne::create();
    listener->setSwallowTouches(true);

    listener->onTouchBegan = [&](cocos2d::Touch* touch, cocos2d::Event* event)
    {
        cocos2d::Vec2 p = touch->getLocation();
        cocos2d::Rect rect = this->getBoundingBox();

        if(rect.containsPoint(p))
        {
        	// Brake is pressed
        	pressed_ = true;
            return true; // to indicate that we have consumed it.
        }

        return false; // we did not consume this event, pass thru.
    };

    listener->onTouchEnded = [=](cocos2d::Touch* touch, cocos2d::Event* event)
    {
    	Pedal::touchEvent(touch);
    };

    cocos2d::Director::getInstance()->getEventDispatcher()->addEventListenerWithFixedPriority(listener, 30);
}

void Pedal::touchEvent(cocos2d::Touch* touch)
{
	pressed_ = false;
}
