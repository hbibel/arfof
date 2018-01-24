/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <thread>

#include <actionlib/client/simple_action_client.h>

#include <QApplication>
#include <QDialog>
#include <QListWidget>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QProcess>
#include <QStringListModel>
#include <QTableView>

#include "workcell_controller.h"
#include "ui/agent_list_item.h"
#include "ui/add_agent_dialog.h"
#include "ui/add_step_dialog.h"
#include "ui/AgentType.h"
#include "ui/agent_list_item_model.h"
#include "ui/step_model.h"

class AddStepDialog;

/**
 * @section DESCRIPTION
 * The main UI for the factory floor manager.
 *
 */
class FFMUI : public QApplication
{
public:
	/**
	 * The constructor for the FFMUI.
	 * @param argc                This is passed to default
	 *                            <a href="http://doc.qt.io/qt-5/qapplication.html">QApplication</a> constructor.
	 * @param argv                This is passed to default
	 *                            <a href="http://doc.qt.io/qt-5/qapplication.html">QApplication</a> constructor.
	 * @param workcell_controller The WorkcelLController which can be operated by this application.
	 * @return
	 */
	FFMUI(int argc, char** argv, WorkcellController *workcell_controller);
	~FFMUI() {
	}
	
	/**
	 * Starts the UI.
	 * @return
	 */
	int exec();
	
	/**
	 * Creates an agent and adds it to the UI.
	 * @param type      What type of agent is to be created.
	 * @param name      The name as it appears in the UI.
	 * @param node_name The ROS node name.
	 */
	void addAgent(AgentType type, QString name, QString node_name);
	
	/**
	 * Creates a step and adds it to the UI.
	 * @param agent_controller_ptr The controller the step will be assigned to
	 * @param parameters           The parameters for the step
	 */
	void addStep(std::shared_ptr<AgentController> agent_controller_ptr, std::shared_ptr<Capability::Parameters> parameters_ptr);
	
	/**
	 * Iterates over all agent items and calls their updateStatus method.
	 */
	void updateAgentList();
	
	/**
	 *
	 * @return a vector containing pointers to the agents in the workcell
	 */
	const std::vector<std::shared_ptr<AgentController>> getAgents();

private:
	// Currently this program only supports one workcell named "workcell_1".
	WorkcellController *_workcell_controller_ptr;
	
	QTableView *_agent_list_view;
	AddAgentDialog *_add_agent_dialog;
	QMainWindow *_window;
	AgentListModel *_agent_list_item_model;
	StepModel* _step_model;
	QPushButton *_add_step_button;
	QPushButton *_open_rqt_console_button;
	QProcess *_rqt_process;
	
	/**
	 * Creates a new AddAgentDialog where the user can set up a new agent.
	 */
	void onAddAgentButtonClicked();
	
	/**
	 * Creates a new AddStepDialog where the user can create a new step for an agent to handle.
	 */
	void onAddStepButtonClicked();
};
