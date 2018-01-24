/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGridLayout>

#include "workcell_controller.h"
#include "ui/AgentType.h"

class FFMUI;

/**
 * This dialog guides a user through adding a new agent.
 */
class AddAgentDialog : public QDialog {
public:
	/**
	 * Constructs a dialog which can create and add agents to a workcell.
	 * @param parent              The parent widget. See the Qt documentation for a detailed description.
	 * @param f                   The window flags. See the Qt documentation for a detailed description.
	 * @param ui                  The UI will handle the newly created agent.
	 * @return
	 */
	AddAgentDialog(QWidget *parent, Qt::WindowFlags f, FFMUI *ui);
	
	/**
	 * Once this dialog is accepted, the new agent is created by the UI.
	 */
	virtual void accept() override;

private:
	// UI elements
	QGridLayout *_add_agent_layout;
	QWidget *_set_up_type_widget;
	QGridLayout *_set_up_type_layout;
	QWidget *_name_widget;
	QFormLayout *_name_widget_layout;
	QDialogButtonBox *_dialog_navigation_buttons;
	QPushButton *_next_button;
	QPushButton *_cancel_button;
	QPushButton *_previous_button;
	QComboBox *_select_agent_type_dropdown;
	QLineEdit *_name_edit;
	QLineEdit *_node_name_edit;
	
	FFMUI *_ui;
	
	AgentType _selected_agent_type;
	
	/**
	 * Step 1 of creating an agent: Select the type of agent
	 * Sets up the dropdown menu from which the user can select the type of the agent to be created.
	 */
	void setUpAgentTypeSelectionLayout();
	/**
	 * Step 2 of creating an agent: Name it
	 * Sets up QLineEdits for the agent name and the node name
	 */
	void setUpAgentDefinitionLayout();
};
