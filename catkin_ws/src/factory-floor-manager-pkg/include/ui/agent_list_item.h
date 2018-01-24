/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <memory>

#include <QStandardItem>

#include "agent_controller.h"
#include "user_roles.h"

typedef std::shared_ptr<AgentController> AgentPtr;

Q_DECLARE_METATYPE(AgentPtr);

/**
 * @section DESCRIPTION
 * This class represents an item in the agent list in the FFMUI.
 */
class AgentListItem : public QStandardItem {
public:
	/**
	 * The constructor for an AgentListItem.
	 * @param name                 The name of the item. This is the name of the agent as it appears in the UI.
	 * @param agent_controller_ptr A pointer to an AgentController object as it is used throughout the program.
	 * @return
	 */
	AgentListItem(QString name, std::shared_ptr<AgentController> agent_controller_ptr);

	/**
	 * See the <a href="http://doc.qt.io/qt-5/qstandarditem.html">Qt Documentation</a> for a detailed description.
	 * @param role
	 * @return
	 */
	virtual QVariant data(int role) const override;
	
	/**
	 * Set the background color of the UI item based on the status.
	 * If the agent is idle, it appears blue, if it is busy it appears yellow and if it is disconnected, it will
	 * appear red.
	 */
	void updateStatus();
	
	virtual int type() const override;

private:
	QString _name_in_ui;
	std::shared_ptr<AgentController> _agent_controller_ptr;
};