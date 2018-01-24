/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <QLabel>
#include <QPushButton>
#include <QStandardItemModel>
#include <QtWidgets>

#include "ui/add_agent_dialog.h"
#include "ui/ffm_ui.h"

// Converts a string to a valid node name, only containing lower case characters, digits and underscores
// example: from = "%$ Agent 45" will be converted to "_agent_45"
QString convert_to_node_name(QString from);

AddAgentDialog::AddAgentDialog(QWidget *parent, Qt::WindowFlags f, FFMUI *ui)
				: QDialog(parent, f), _ui(ui) {
	this->setWindowTitle("Add Agent");
	resize(640, 480);
	
	// The overall layout
	_add_agent_layout = new QGridLayout(this);
	
	// set up the buttons
	_dialog_navigation_buttons = new QDialogButtonBox(this);
	_next_button = _dialog_navigation_buttons->addButton("Next", QDialogButtonBox::AcceptRole);
	_cancel_button = _dialog_navigation_buttons->addButton("Cancel", QDialogButtonBox::RejectRole);
	connect(_cancel_button, SIGNAL(clicked()), this, SLOT(close()));
	_previous_button = _dialog_navigation_buttons->addButton("Previous", QDialogButtonBox::RejectRole);
	
	_add_agent_layout->addWidget(_dialog_navigation_buttons, 1, 0, Qt::AlignBottom);
	
	// Start with step 1: select the agent type
	setUpAgentTypeSelectionLayout();
	
	this->setLayout(_add_agent_layout);
}

void AddAgentDialog::setUpAgentTypeSelectionLayout() {
	delete _name_widget;
	
	_set_up_type_widget = new QWidget(this);
	_set_up_type_layout = new QGridLayout(_set_up_type_widget);
	_set_up_type_widget->setLayout(_set_up_type_layout);
	_select_agent_type_dropdown = new QComboBox(_set_up_type_widget);
	_select_agent_type_dropdown->addItem("Select Agent Type");
	_select_agent_type_dropdown->addItem("Robotino");
	/* *************************
	 * ADD MORE AGENT TYPES HERE
	 * *************************/
	
	// Set up the dropdown menu
	QStandardItemModel* model =
					qobject_cast<QStandardItemModel*>(_select_agent_type_dropdown->model());
	QModelIndex firstIndex = model->index(0, _select_agent_type_dropdown->modelColumn(),
																				_select_agent_type_dropdown->rootModelIndex());
	QStandardItem* firstItem = model->itemFromIndex(firstIndex);
	firstItem->setSelectable(false); // sets the first item ("Select Agent Type") unselectable
	
	// Set up the dialog layout
	_add_agent_layout->addWidget(_set_up_type_widget, 0, 0, Qt::AlignTop);
	_set_up_type_layout->addWidget(_select_agent_type_dropdown);
	_previous_button->setDisabled(true);
	_next_button->setDisabled(true);
	_next_button->setText("Next");
	_next_button->disconnect();
	connect(_select_agent_type_dropdown, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
					[=](){_next_button->setDisabled(false);});
	connect(_next_button, &QPushButton::clicked, this, &AddAgentDialog::setUpAgentDefinitionLayout);
}

void AddAgentDialog::setUpAgentDefinitionLayout() {
	if (_select_agent_type_dropdown->currentIndex() == 1) {
		_selected_agent_type = AgentType::ROBOTINO;
	}
	/* *************************
	 * ADD MORE AGENT TYPES HERE
	 * *************************/
	
	// Delete the previous name_widget_layout
	_add_agent_layout->removeWidget(_select_agent_type_dropdown);
	_select_agent_type_dropdown->deleteLater();
	
	// Set up the new name_widget_layout for this step
	_name_widget = new QWidget(this);
	_name_widget_layout = new QFormLayout(_name_widget);
	_name_edit = new QLineEdit(_name_widget);
	_name_widget_layout->addRow(new QLabel("Agent Name:", (_name_widget)), _name_edit);
	_node_name_edit = new QLineEdit(_name_widget);
	_name_widget_layout->addRow(new QLabel("Node Name:", (_name_widget)), _node_name_edit);
	connect(_name_edit, &QLineEdit::textChanged, _node_name_edit, [&](){
		_node_name_edit->setText(convert_to_node_name(_name_edit->text()));
	});
	_name_widget->setLayout(_name_widget_layout);
	_add_agent_layout->addWidget(_name_widget, 0, 0);
	
	// Update the previous / next / cancel buttons
	_previous_button->setDisabled(false);
	_next_button->setText("Finish");
	
	delete _set_up_type_widget;
	
	connect(_previous_button, &QPushButton::clicked, this, &AddAgentDialog::setUpAgentTypeSelectionLayout);
	disconnect(_next_button, &QPushButton::clicked, this, &AddAgentDialog::setUpAgentDefinitionLayout);
	connect(_next_button, &QPushButton::clicked, this, &QDialog::accept);
}

void AddAgentDialog::accept() {
	// Notify the UI of the new agent. The actual creation of the agent is done in the FMMUI class.
	_ui->addAgent(_selected_agent_type, _name_edit->text(), _node_name_edit->text());
	QDialog::accept();
}

QString convert_to_node_name(QString from) {
	QString result;
	for (QChar c : from) {
		ushort unicode_value = c.unicode();
		// No umlauts or hyphenated letters
		if ((unicode_value >= 0x0041) && (unicode_value <= 0x005A) // A..Z
				|| (unicode_value >= 0x0061) && (unicode_value <= 0x007A) // a..z
				|| c.isDigit()) {
			result.append(c.toLower());
		}
		else {
			if (result.length() == 0 || result[result.length() - 1].isLetter()) {
				result.append("_");
			}
		}
	}
	if (result.length() == 0) {
		result = "generic_node_name";
	}
	return result;
}