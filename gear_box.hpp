/**
* @author: Clark Li <clark.li86@gmail.com>
* @date: 26/12/2014
* Implement a gear box to change direction - forward or backward
*/

#ifndef GEAR_BOX_HPP
#define GEAR_BOX_HPP
#include <cocos2d.h>
#include "defs.h"

/// Control the gear with a touchable sprite
class GearBox : public cocos2d::Sprite{
public:
	enum class Gear {
	    FORWARD  = 1,
	    BACKWARD = 2
	};
    /// Return current gear
    Gear gear() const { return gear_; }

    static GearBox* create();

	void initOptions();

	void addEvents();
	void touchEvent(cocos2d::Touch* touch);
private:

    /// Sprite to draw the gear box
    cocos2d::Sprite * sprite_ { nullptr };
    /// Current gear
    Gear gear_ { Gear::FORWARD };
};

#endif // GEAR_BOX_HPP
