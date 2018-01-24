/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <memory>

#include <QStandardItemModel>
#include <QStringListModel>

#include "capability.h"

typedef std::shared_ptr<Capability> CapabilityPtr;

Q_DECLARE_METATYPE(CapabilityPtr);

/**
 * An item for a model containing a Capability.
 */
class CapabilityItem : public QStandardItem {
public:
	CapabilityItem(CapabilityPtr capability_ptr);
	~CapabilityItem(){}
	
	virtual QVariant data(int role) const override;
	
private:
	CapabilityPtr _capability_ptr;
};