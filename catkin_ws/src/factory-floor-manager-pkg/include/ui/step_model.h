/**
 * @file
 * @author  Hannes Bibel <bibel@in.tum.de>
 */

#pragma once

#include <QAbstractListModel>

#include "step.h"

typedef std::shared_ptr<Step> StepPtr;

Q_DECLARE_METATYPE(StepPtr);

/**
 * The StepModel is a Qt Model that handles a list of steps and how they are displayed and updated.
 */
class StepModel : public QAbstractListModel {
public:
	/**
	 * @param steps_vec The list of steps that are to be displayed.
	 * @param parent    The parent object.
	 */
	StepModel(const std::vector<StepPtr>& steps_vec,
						QObject *parent = Q_NULLPTR);
	
	/**
	 * @param parent
	 * @return The size of the list of steps.
	 */
	virtual int rowCount(const QModelIndex& parent) const override;
	
	/**
	 * For the DisplayRole, this returns the description of the step as defined in the Step class or one of its
	 * subclasses.
	 * For the StepRole this returns a shared_ptr to the step.
	 *
	 * @param index The vector index of the step.
	 * @param role  Qt::DisplayRole or UserRoles::StepRole
	 */
	virtual QVariant data(const QModelIndex& index, int role) const override;
	
	/**
	 * Emits the dataChanged() signal for all items.
	 */
	void update();

private:
	const std::shared_ptr<const std::vector<StepPtr>> _steps_vec;
};