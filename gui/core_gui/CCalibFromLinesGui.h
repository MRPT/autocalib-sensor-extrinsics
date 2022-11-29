#pragma once

#include <observation_tree/CObservationTreeGui.h>
#include <calib_solvers/CCalibFromLines.h>
#include <interfaces/CLinesObserver.h>
#include <interfaces/CCorrespLinesObserver.h>

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * Provides a GUI wrapper around the core calibration from lines classes.
 *
 * Inherits from CCalibFromLines.
 */

class CCalibFromLinesGui : public CCalibFromLines
{
public:

	/**
	 * Constructor
	 * \param model The sync rawlog model.
	 * \param params The parameters for the calibration.
	 */
	CCalibFromLinesGui(CObservationTree *model, TCalibFromLinesParams *params);

	~CCalibFromLinesGui();

	/** Runs line segmentation on all the image observations in the model. */
	void extractLines();

	/** Runs line matching. */
	void matchLines();

	/** Adds observer to list of text observers. */
	void addTextObserver(CTextObserver *observer);

	/** Adds observer to list of line observers. */
	void addLinesObserver(CLinesObserver *observer);

	/** Adds observer to list of matched lines observers. */
	void addCorrespLinesObserver(CCorrespLinesObserver *observer);

	/** Notifies text observers with a message. */
	void publishText(const std::string &msg);

	/** Notifies observers with the extracted lines.
	 * \param sensor_id id of the sensor whose observation the lines were extracted from.
	 * \param sync_obs_id id of the observation in the synchronized rawlog.
	 */
	void publishLines(const int &sensor_id, const int &sync_obs_id);

	/** Notifies observers with the lines that were matched between each pair of sensors in a sync set, if any.
	 * \param obs_set_id the id of the synchronized observation set.
	 */
	void publishCorrespLines(const int &obs_set_id);

	/** Returns the status of the calibration progress. */
	CalibFromLinesStatus calibStatus();


private:

	/** List of text observers to be notified about the progress. */
	std::vector<CTextObserver*> m_text_observers;

	/** List of observers to be notified about the extracted lines. */
	std::vector<CLinesObserver*> m_lines_observers;

	/** List of observers to be notified about the matched lines. */
	std::vector<CCorrespLinesObserver*> m_corresp_lines_observers;
};
