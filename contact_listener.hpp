#ifndef  CONTACT_LISTENER_HPP
#define  CONTACT_LISTENER_HPP
#include "Box2D/Box2D.h"
#include "cocos2d.h"
#include "extensions/cocos-ext.h"
#import <vector>
#import <algorithm>

struct Contact {
    b2Fixture *fixtureA;
    b2Fixture *fixtureB;
    bool operator==(const Contact& other) const {
        return (fixtureA == other.fixtureA) && (fixtureB == other.fixtureB);
    }
};

class ContactListener : public b2ContactListener
{
public:
	/// Called when two fixtures begin to touch.
	virtual void BeginContact(b2Contact* contact) {
		Contact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
		contacts_.push_back(myContact);
	}

	virtual void EndContact(b2Contact* contact) {
		Contact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
	    std::vector<Contact>::iterator pos;
	    // Remove contact
	    pos = std::find(contacts_.begin(), contacts_.end(), myContact);
	    if (pos != contacts_.end()) {
	    	contacts_.erase(pos);
	    }
	    // Check if car collides with obstacles
	    int tag1 = 0, tag2 = 0;
	    if(contact->GetFixtureA() && contact->GetFixtureA()->GetBody() && contact->GetFixtureA()->GetBody()->GetUserData()) {
	    	tag1 = reinterpret_cast<cocos2d::Node*>(contact->GetFixtureA()->GetBody()->GetUserData())->getTag();
	    }
	    if(contact->GetFixtureB() && contact->GetFixtureB()->GetBody() && contact->GetFixtureB()->GetBody()->GetUserData()) {
			tag2 = reinterpret_cast<cocos2d::Node*>(contact->GetFixtureB()->GetBody()->GetUserData())->getTag();
		}
	    if(    (tag1 == CAR_TAG && tag2 != CAR_TAG)
	    	|| (tag1 != CAR_TAG && tag2 == CAR_TAG)
	      ) {
	    	CCLOGERROR("contact with car");
	    }
	}
private:
	std::vector<Contact> contacts_;
};
#endif // CONTACT_LISTENER_HPP

