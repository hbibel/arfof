/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "agent_list_item.h"

AgentListItem::AgentListItem(QString name, std::shared_ptr<AgentController> agent_controller_ptr)
				: _name_in_ui(name), _agent_controller_ptr(agent_controller_ptr) {
	
}

QVariant AgentListItem::data(int role) const {
	if (Qt::DisplayRole == role) {
		// Set the text to be displayed as the _name_in_ui string.
		return _name_in_ui + " (" + QString::fromStdString(_agent_controller_ptr->getType()) + ")";
	}
	if (role == UserRoles::AgentPointerRole) {
		return QVariant::fromValue(_agent_controller_ptr);
	}
	return QStandardItem::data(role);
}

void AgentListItem::updateStatus() {
	QVariant bg_color;
	switch (_agent_controller_ptr->getStatus()) {
		case AgentController::Status::IDLE: {
			bg_color = QColor(Qt::blue);
			break;
		}
		case AgentController::Status::MOVING: {
			bg_color = QColor(Qt::yellow);
			break;
		}
		case AgentController::Status::DISCONNECTED: {
			bg_color = QColor(Qt::red);
			break;
		}
	}
	this->setData(bg_color, Qt::BackgroundColorRole);
	emitDataChanged();
}

int AgentListItem::type() const {
	// If more types are required it might make sense to create a user_type enum.
	return QStandardItem::UserType + 1;
}
