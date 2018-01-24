/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <functional>
#include <boost/shared_ptr.hpp>

// Every man's and robot's capabilities have limits, which have to be modeled. This is not done yet in this class.
/**
 * @section DESCRIPTION
 * This is the base class for all capabilities.
 * A capability models what an agent can do, such as moving or lifting weights. Every class inheriting from this
 * base class should also contain a nested struct called "Parameters", which inherits from `Capability::Parameters`.
 */
class Capability {
public:
	/**
	 * The Parameters for a capability.
	 * Parameters can be everything that describes a capability. A MakeCoffeeCapability for example might have the
	 * following parameters:
	 * \code{.cpp}
   * struct Parameters : public Capability::Parameters {
	 * enum Strength {
	 * 	Light,
	 * 	Medium,
	 * 	Strong
	 * };
	 * Strength strength;
	 * bool with_milk;
	 * int number_of_sugar_cubes;
	 * };
   * \endcode
	 */
	struct Parameters {
		virtual const std::string getCapabilityName() {
			return base_class_name;
		}

	private:
		static const std::string base_class_name;
	};

	/**
	 * Compares the name of the capability to a given name. Mind that the name of a capability is not an attribute of
	 * a particular instance of a Capability, but a class attribute.
	 * @param capability_name Some string
	 * @return `true`, if `capability_name` is the name of this capability.
	 */
	virtual bool isNamed(const std::string& capability_name) {
		return getName() == capability_name;
	};

	/**
	 * Performing a capability will trigger a callback.
	 */
	virtual void perform(std::shared_ptr<Parameters> p) = 0;

	/**
	 * @return The (static) name of the capability. For the base class, this name is "capability_base_class".
	 */
	virtual const std::string& getName();
};