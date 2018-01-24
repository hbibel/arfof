#include <QCheckBox>
#include <QInputDialog>

#include "add_step_dialog.h"

AddStepDialog::AddStepDialog(FFMUI *ui,
														 const std::vector<std::shared_ptr<AgentController>>& agents,
														 AgentListModel *agent_list_model,
														 QWidget *parent,
														 Qt::WindowFlags f) : QDialog(parent, f), _ui { ui }, _agents { agents },
																									_agent_list_model { agent_list_model } {
	this->setWindowTitle("Add Step");
	resize(640, 480);
	
	// The overall layout
	_dialog_layout = new QGridLayout(this);
	// set up the buttons
	_dialog_navigation_buttons = new QDialogButtonBox(this);
	_next_button = _dialog_navigation_buttons->addButton("Next", QDialogButtonBox::ActionRole);
	_cancel_button = _dialog_navigation_buttons->addButton("Cancel", QDialogButtonBox::RejectRole);
	connect(_cancel_button, SIGNAL(clicked()), this, SLOT(close()));
	connect(_next_button, SIGNAL(clicked()), this, SLOT(selectCapability()));
	_dialog_layout->addWidget(_dialog_navigation_buttons, 1, 0, Qt::AlignBottom);
	
	// Agent selection layout
	_select_agent_widget = new QWidget(this);
	_select_agent_layout = new QGridLayout(this);
	_select_agent_widget->setLayout(_select_agent_layout);
	_dialog_layout->addWidget(_select_agent_widget);
	
	_select_label = new QLabel("Select Agent:", _select_agent_widget);
	_select_agent_layout->addWidget(_select_label);
	_agent_list_view = new QListView(_select_agent_widget);
	_agent_list_view->setModel(_agent_list_model);
	// default selection: first item in the list
	_agent_list_view->setCurrentIndex(_agent_list_model->index(0, 0));
	_select_agent_layout->addWidget(_agent_list_view);
	
	this->setLayout(_dialog_layout);
}

void AddStepDialog::selectCapability() {
	// shows a dialog from which the user can select a capability from a list of capabilities of the agent the user has
	// chosen in the previous step.
	
	_select_capability_widget = new QWidget(this);
	_select_capability_layout = new QGridLayout(this);
	_select_capability_widget->setLayout(_select_capability_layout);
	
	_select_label = new QLabel("Select Capability:", _select_capability_widget);
	_select_capability_layout->addWidget(_select_label);
	QItemSelectionModel *selection_model = _agent_list_view->selectionModel();
	QModelIndex selected_index = selection_model->currentIndex();
	AgentListItem *agent_item = static_cast<AgentListItem*>(_agent_list_model->itemFromIndex(selected_index));
	QVariant agent_ptr_qvar = agent_item->data(UserRoles::AgentPointerRole);
	if (agent_ptr_qvar.canConvert<AgentPtr>()) {
		_agent_ptr = agent_ptr_qvar.value<AgentPtr>();
		const std::vector<CapabilityPtr> capabilities = _agent_ptr->getCapabilities();
		_capability_list_model = new QStandardItemModel(0, 1, this);
		for (CapabilityPtr capability_ptr : capabilities) {
			CapabilityItem *capability_item = new CapabilityItem(capability_ptr);
			_capability_list_model->appendRow(capability_item);
		}
		_capability_list_view = new QListView(_select_capability_widget);
		_capability_list_view->setModel(_capability_list_model);
		_capability_list_view->setCurrentIndex(_capability_list_model->index(0, 0));
		_select_capability_layout->addWidget(_capability_list_view);
		
		// replace the _select_agent_widget by the _select_capability_widget
		_dialog_layout->removeWidget(_select_agent_widget);
		delete _select_agent_widget;
		_dialog_layout->addWidget(_select_capability_widget);
	}
	disconnect(_next_button, SIGNAL(clicked()), this, SLOT(selectCapability()));
	connect(_next_button, SIGNAL(clicked()), this, SLOT(setUpParameters()));
}

void AddStepDialog::setUpParameters() {
	_set_up_parameters_widget = new QWidget(this);
	_set_up_parameters_layout = new QGridLayout(this);
	
	_next_button->disconnect();
	QItemSelectionModel *selection_model = _capability_list_view->selectionModel();
	QModelIndex selected_index = selection_model->currentIndex();
	CapabilityItem *capability_item = static_cast<CapabilityItem*>(_capability_list_model->itemFromIndex(selected_index));
	QVariant capability_ptr_qvar = capability_item->data(UserRoles::CapabilityRole);
	if (capability_ptr_qvar.canConvert<CapabilityPtr>()) {
		CapabilityPtr capability_ptr = capability_ptr_qvar.value<CapabilityPtr>();
		// I haven't found a prettier way to check for the type of capability and dynamically generate the parameters
		if (capability_ptr->getName() == "two_d_move") {
			_set_up_parameters_layout->addWidget(new QLabel("move_x:", _set_up_parameters_widget));
			QLineEdit *x_input = new QLineEdit(_set_up_parameters_widget);
			x_input->setValidator(new QDoubleValidator(_set_up_parameters_widget));
			x_input->setText("0.0");
			_set_up_parameters_layout->addWidget(x_input);
			_set_up_parameters_layout->addWidget(new QLabel("move_y:", _set_up_parameters_widget));
			QLineEdit *y_input = new QLineEdit(_set_up_parameters_widget);
			y_input->setValidator(new QDoubleValidator(_set_up_parameters_widget));
			y_input->setText("0.0");
			_set_up_parameters_layout->addWidget(y_input);
			_set_up_parameters_layout->addWidget(new QLabel("rotation:", _set_up_parameters_widget));
			QLineEdit *rot_input = new QLineEdit(_set_up_parameters_widget);
			rot_input->setValidator(new QDoubleValidator(_set_up_parameters_widget));
			rot_input->setText("0.0");
			_set_up_parameters_layout->addWidget(rot_input);
			connect(_next_button, &QPushButton::clicked, [=](){
				AddStepDialog::createTwoDMoveCapabilities(x_input->text().toFloat(), y_input->text().toFloat(), rot_input->text().toFloat());
			});
		}
	}
	_set_up_parameters_widget->setLayout(_set_up_parameters_layout);
	
	_dialog_layout->removeWidget(_select_capability_widget);
	delete _select_capability_widget;
	_dialog_layout->addWidget(_set_up_parameters_widget);
}

TwoDMoveCapabilityParameters AddStepDialog::createTwoDMoveCapabilities(float x, float y, float phi) {
	_parameters = std::make_shared<TwoDMoveCapabilityParameters>(x, y, phi);
	this->accept();
}

void AddStepDialog::accept() {
	_ui->addStep(_agent_ptr, _parameters);
	QDialog::accept();
}

// include the meta object compiled code
#include "moc_add_step_dialog.cpp"