/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <QStandardItemModel>

/**
 * @section DESCRIPTION
 * The model for the agent list.
 */
class AgentListModel : public QStandardItemModel {
public:
	AgentListModel(int rows, int columns, QObject * parent = 0);

private:
};