/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include "capability.h"

/**
 * The capability of an agent to move in a two-dimensional environment, like on a floor.
 */
class TwoDMoveCapability : public Capability {
public:
	struct Parameters : public Capability::Parameters {
		Parameters(float x, float y, float rot) : to_x(x), to_y(y), to_rotation(rot) {}
		// Since this has three parameters I really wonder if this class would better be called ThreeDMoveCapability
		float to_x;
		float to_y;
		float to_rotation;

		/**
		 * @return "two_d_move"
		 */
		const std::string getCapabilityName() override;

	private:
		static const std::string capability_name; // "two_d_move"
	};
	
	// if you change one of these lines, change the corresponding lines below the class definition as well.
	typedef TwoDMoveCapability::Parameters TwoDMoveCapabilityParameters;
	typedef std::function<void(std::shared_ptr<TwoDMoveCapabilityParameters>)> TwoDMoveCapabilityCallback;

	/**
	 * To construct a capability, you have to supply it with a function that executes this capability in the real world.
	 */
	TwoDMoveCapability(TwoDMoveCapabilityCallback callback);
	~TwoDMoveCapability();

	/**
	 * @return "two_d_move"
	 */
	const std::string& getName();

	/**
	 * This method first checks whether the parameters fit to the capability. If not, an error is printed to rosout.
	 * If yes, the callback provided with the construction of this capability is called.
	 * @param p If `p`'s <Capability::Parameters::get_capability_name>"()" returns "two_d_move", the callback will be
	 * invoked.
	 */
	void perform(std::shared_ptr<Capability::Parameters> p) override;

private:
	TwoDMoveCapabilityCallback _callback;
};

// if you change one of these lines, change the corresponding lines within the class definition as well.
typedef TwoDMoveCapability::Parameters TwoDMoveCapabilityParameters;
typedef std::function<void(std::shared_ptr<TwoDMoveCapabilityParameters>)> TwoDMoveCapabilityCallback;
