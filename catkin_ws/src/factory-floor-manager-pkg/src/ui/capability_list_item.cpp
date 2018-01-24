/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <include/ui/user_roles.h>
#include "capability_list_item.h"

CapabilityItem::CapabilityItem(CapabilityPtr capability_ptr) : _capability_ptr { capability_ptr } {
	
}

QVariant CapabilityItem::data(int role) const {
	if (role == UserRoles::CapabilityRole) {
		return QVariant::fromValue(_capability_ptr);
	}
	if (role == Qt::DisplayRole) {
		return QString::fromStdString(_capability_ptr->getName());
	}
	return QStandardItem::data(role);
}
