/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#include <algorithm>

#include <QHeaderView>
#include <QPushButton>
#include <QTextEdit>

#include "robotino_controller.h"

#include "ui/ffm_ui.h"

FFMUI::FFMUI(int argc, char** argv, WorkcellController *workcell_controller)
				: QApplication(argc, argv), _workcell_controller_ptr(workcell_controller)
{}

int FFMUI::exec() {
	_window = new QMainWindow;
	QWidget *main_window_widget = new QWidget;
	
	QHBoxLayout *main_layout = new QHBoxLayout(main_window_widget);
	// The agent list
	QWidget *left_widget = new QWidget; // widget containing the agent_column_layout
	QHBoxLayout *agent_column_layout = new QHBoxLayout(left_widget); // Layout including agent list and buttons
	_agent_list_view = new QTableView(left_widget); // list of agents
	
	_agent_list_item_model = new AgentListModel(0, 1);
	_agent_list_view->setModel(_agent_list_item_model);
	_agent_list_view->horizontalHeader()->setStretchLastSection(true);
	
	agent_column_layout->addWidget(_agent_list_view);
	QVBoxLayout *agent_action_button_list = new QVBoxLayout(left_widget); // buttons for adding an agent, ...
	QIcon add_icon = QIcon::fromTheme("list-add");
	QPushButton *add_agent_button = new QPushButton(left_widget); // button to click to add a new agent
	add_agent_button->setIcon(add_icon);
	connect(add_agent_button, &QPushButton::clicked, this, &FFMUI::onAddAgentButtonClicked);
	agent_action_button_list->addWidget(add_agent_button);
	agent_action_button_list->setAlignment(add_agent_button, Qt::AlignTop);
	agent_column_layout->addLayout(agent_action_button_list);
	QSizePolicy spLeft(QSizePolicy::Preferred, QSizePolicy::Preferred);
	spLeft.setHorizontalStretch(1);
	left_widget->setSizePolicy(spLeft);
	left_widget->setLayout(agent_column_layout);
	main_layout->addWidget(left_widget);
	
	// The list of currently executed steps:
	QWidget *step_widget = new QWidget;
	QSizePolicy spRight(QSizePolicy::Preferred, QSizePolicy::Preferred);
	spRight.setHorizontalStretch(1);
	step_widget->setSizePolicy(spRight);
	_step_model = new StepModel(_workcell_controller_ptr->getExecutingSteps(), step_widget);
	QListView* step_list_view = new QListView(step_widget);
	step_list_view->setModel(_step_model);
	
	QHBoxLayout *step_layout = new QHBoxLayout(step_widget);
	step_layout->addWidget(step_list_view);
	_add_step_button = new QPushButton(step_list_view);
	_add_step_button->setIcon(add_icon);
	connect(_add_step_button, &QPushButton::clicked, this, &FFMUI::onAddStepButtonClicked);
	step_layout->addWidget(_add_step_button);
	step_layout->setAlignment(_add_step_button, Qt::AlignTop);
	main_layout->addWidget(step_widget);
	
	_open_rqt_console_button = new QPushButton("Open rqt_console");
	_rqt_process = new QProcess;
	connect(_open_rqt_console_button, &QPushButton::clicked, [=]{
		_rqt_process->start("rqt_console");
	});
	main_layout->addWidget(_open_rqt_console_button);
	main_layout->setAlignment(_open_rqt_console_button, Qt::AlignTop);
	
	main_window_widget->setLayout(main_layout);
	_window->setCentralWidget(main_window_widget);
	_window->showMaximized();
	
	return QApplication::exec();
}

void FFMUI::onAddAgentButtonClicked() {
	_add_agent_dialog = new AddAgentDialog(_window, Qt::Dialog, this);
	_add_agent_dialog->setAttribute(Qt::WA_DeleteOnClose);
	_add_agent_dialog->show();
}

void FFMUI::addAgent(AgentType type, QString name, QString node_name) {
	switch (type) {
		case ROBOTINO: {
			std::string node_name_std = node_name.toStdString();
			auto robotino_controller_ptr = std::make_shared<RobotinoController>(node_name_std, _workcell_controller_ptr->getNodeHandle());
			_workcell_controller_ptr->addAgent(robotino_controller_ptr);
			AgentListItem *item = new AgentListItem(name, robotino_controller_ptr);
			_agent_list_item_model->appendRow(item);
			break;
		}
		default:
			ROS_ERROR("%s, %d: No agent has been added.", __FILE__, __LINE__);
			break;
	}
}

void FFMUI::updateAgentList() {
	for (int i = 0; i < _agent_list_item_model->rowCount(); i++) {
		AgentListItem *item = static_cast<AgentListItem *>(_agent_list_item_model->item(0,0));
		item->updateStatus();
	}
}

void FFMUI::onAddStepButtonClicked() {
	AddStepDialog *_add_step_dialog = new AddStepDialog(this, _workcell_controller_ptr->getAgents(), _agent_list_item_model);
	_add_step_dialog->setAttribute(Qt::WA_DeleteOnClose);
	_add_step_dialog->show();
}

const std::vector<std::shared_ptr<AgentController>> FFMUI::getAgents() {
	return _workcell_controller_ptr->getAgents();
}

void FFMUI::addStep(std::shared_ptr<AgentController> agent_controller_ptr, std::shared_ptr<Capability::Parameters> parameters_ptr) {
	std::shared_ptr<Step> step_ptr;
	if (parameters_ptr->getCapabilityName() == "two_d_move" && agent_controller_ptr->getType() == "Robotino") {
		step_ptr = std::make_shared<RobotinoTwoDMoveStep>(std::static_pointer_cast<RobotinoController>(agent_controller_ptr)
						, std::static_pointer_cast<TwoDMoveCapabilityParameters>(parameters_ptr));
	}
	_workcell_controller_ptr->addStep(step_ptr);
	_step_model->update();
}

