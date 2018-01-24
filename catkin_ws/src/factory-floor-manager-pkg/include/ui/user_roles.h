#pragma once

/**
 * Special roles to be used Qt's data() methods. These custom roles start at 1001 to avoid clashes with
 * predefined roles.
 */
enum UserRoles {
	AgentPointerRole = 1001,
	CapabilityRole,
	StepRole
};