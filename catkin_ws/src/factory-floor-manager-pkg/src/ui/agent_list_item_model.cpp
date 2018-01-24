/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include "agent_list_item_model.h"

AgentListModel::AgentListModel(int rows, int columns, QObject *parent)
				: QStandardItemModel(rows, columns, parent) {
	QStringList header_labels;
	header_labels << "Agent";
	this->setHorizontalHeaderLabels(header_labels);
}
