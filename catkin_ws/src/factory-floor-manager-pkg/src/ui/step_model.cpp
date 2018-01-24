#include "ui/user_roles.h"
#include "robotino_two_d_move_step.h"
#include "step_model.h"

StepModel::StepModel(const std::vector<StepPtr>& steps_vec,
										 QObject *parent)
				: QAbstractListModel(parent),
					_steps_vec(&steps_vec) {}

int StepModel::rowCount(const QModelIndex& parent) const {
	return (int) _steps_vec->size();
}

QVariant StepModel::data(const QModelIndex& index, int role) const {
	StepPtr step_ptr = _steps_vec->at((unsigned long) index.row());
	if (role == Qt::DisplayRole) {
		std::string result_string = step_ptr.get()->getDescription();
		return QString::fromStdString(result_string);
	}
	if (role == UserRoles::StepRole) {
		// Returns a pointer to the step
		return QVariant::fromValue(step_ptr);
	}
	return QVariant();
}

void StepModel::update() {
	QModelIndex top_left = this->index(0);
	QModelIndex bottom_right = this->index((int) _steps_vec->size() - 1);
	emit dataChanged(top_left, bottom_right);
}
