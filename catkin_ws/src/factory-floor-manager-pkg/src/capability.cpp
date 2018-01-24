/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "capability.h"

const std::string Capability::Parameters::base_class_name  = "capability_base_class";

const std::string& Capability::getName()  {
	static std::string name = "capability_base_class";
	return name;
}

