/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <QDialog>
#include <QLabel>
#include <QPushButton>

#include "agent_list_item_model.h"
#include "capability_list_item.h"
#include "ffm_ui.h"
#include "two_d_move_capability.h"

/**
 * This dialog guides a user through adding a new step.
 */
class AddStepDialog : public QDialog {
	Q_OBJECT

public:
	AddStepDialog(FFMUI *ui,
								const std::vector<std::shared_ptr<AgentController>>& agents,
								AgentListModel *agent_list_model,
								QWidget *parent = 0,
								Qt::WindowFlags f = 0);
	
	~AddStepDialog() {}
	
	virtual void accept() override;

private:
	std::vector<std::shared_ptr<AgentController>> _agents;
	AgentPtr _agent_ptr;
	std::shared_ptr<Capability::Parameters> _parameters;
	
	FFMUI* _ui;
	AgentListModel *_agent_list_model;
	QStandardItemModel *_capability_list_model;
	
	QGridLayout *_dialog_layout;
	QDialogButtonBox *_dialog_navigation_buttons;
	QGridLayout *_select_agent_layout;
	QWidget *_select_agent_widget;
	QGridLayout *_select_capability_layout;
	QWidget *_select_capability_widget;
	QGridLayout *_set_up_parameters_layout;
	QWidget *_set_up_parameters_widget;
	QPushButton *_next_button;
	QPushButton *_cancel_button;
	QLabel *_select_label;
	QListView *_agent_list_view;
	QListView *_capability_list_view;
	
	TwoDMoveCapabilityParameters createTwoDMoveCapabilities(float x, float y, float phi);
	
private slots:
	void selectCapability();
	void setUpParameters();
};