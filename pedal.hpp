/**
* @author: Clark Li <clark.li86@gmail.com>
* @date: 27/12/2014
* Pedal, for brake or accelerator
*/

#ifndef PEDAL_HPP
#define PEDAL_HPP
#include <cocos2d.h>
#include "defs.h"

class Pedal : public cocos2d::Sprite {
public:
    bool pressed() const { return pressed_; }

    static Pedal* create(const char * image);

	void initOptions();

	void addEvents();
	void touchEvent(cocos2d::Touch* touch);
private:

    /// Sprite to draw the gear box
    cocos2d::Sprite * sprite_ { nullptr };

    bool pressed_ { false };
};

#endif // PEDAL_HPP
