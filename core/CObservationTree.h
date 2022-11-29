#pragma once

#include "Utils.h"
#include "CObservationTreeItem.h"
#include <interfaces/CTextObserver.h>

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/config/CConfigFile.h>
#include <Eigen/Core>

/**
 * Class for loading, storing, and synchronizing the observations from a rawlog file into a tree.
 * The class also maintains the relative transformations between all the sensors.
 */

class CObservationTree
{
    public:

	    /**
		 * \brief Constructor
		 * \param rawlog_path the path of the rawlog file to load observations from.
		 * \param config_file the master app configuration file.
		 */
	    CObservationTree(const std::string &rawlog_path, const mrpt::config::CConfigFile &config_file);

		/**
		 * \brief Destructor
		 */
		~CObservationTree();

		/**
		 * \brief loadTree loads the contents of the rawlog into the tree.
		 * \param rawlog_filename the rawlog file.
		 */
		void loadTree();

		/**
		 * \brief Returns the path of the rawlog file this model was loaded from.
		 */
		std::string getRawlogPath() const;

		/**
		 * Returns the root item of the tree.
		 */
		CObservationTreeItem *getRootItem() const;

		/** Returns the count of total number of observations found in the rawlog. */
		int getObsCount() const;

		/** Returns the number of unique sensors found in the rawlog. */
		int getNumberOfSensors() const;

		/** Returns a list of the unique sensor labels found in the rawlog. */
		std::vector<std::string> getSensorLabels() const;

		/** Returns the count of observations of each label found in the rawlog. */
		std::vector<int> getCountOfLabel() const;

		/** Returns the poses of the sensors found in the rawlog. */
		std::vector<Eigen::Matrix4f> getSensorPoses() const;

		/** Returns the uncertainties of the sensors found in the rawlog. */
		std::vector<Eigen::Vector2f> getSensorUncertainties() const;

		/** Sets the poses of the sensors found in the rawlog.
		 * \param sensor_poses the new sensor poses (size should match the number of sensors in the model).
		 * \return true or false depending on whether the poses were set.
		 */
		bool setSensorPoses(const std::vector<Eigen::Matrix4f> &sensor_poses);

		/** Sets the pose of an individual sensor in the rawlog.
		 * \param sensor_pose the pose of the sensor.
		 * \param sensor_index the index of the sensor in the rawlog.
		 */
		void setSensorPose(const Eigen::Matrix4f &sensor_pose, const int &sensor_index);

		/** Sets the uncertainties of the sensors found in the rawlog.
		 * \param uncertainties The new uncertainites of the sensors(size should match the number of sensors in the model).
		 * \return bool depending on whether the uncertainties were set.
		 */
		bool setSensorUncertainties(const std::vector<Eigen::Vector2f> &uncertainties);

		/** Set the distance and angular uncertainty of an individual sensor in the rawlog.
		 * \param sensor_uncertainty The distance and angular uncertainty.
		 * \param sensor_index The index of the sensor in the rawlog.
		 */
		void setSensorUncertainty(const Eigen::Vector2f &sensor_uncertainty, const int &sensor_index);

		/** Groups observations together based on their time stamp proximity.
		 * Stores the results back in the same tree.
		 * \param the labels of the sensors that are to be considered for grouping.
		 * \param max_delay Maximum allowable delay between observations.
		 */
		void syncObservations(const std::vector<std::string> &selected_sensor_labels, const int &max_delay);

		/** Returns the indices of the grouped observations with respect to the original tree, grouped by sensor. */
		std::vector<std::vector<int>> getSyncIndices() const;

		/** Retuns the sync index (the index in m_sync_indices[sensor_id] of an item within a set, identified by sensor label. */
		int findSyncIndexFromSet(const int &set_id, const std::string &sensor_label) const;

    protected:

		/** The path of the file the rawlog was loaded from. */
		std::string m_rawlog_path;

		mrpt::config::CConfigFile m_config_file;

		/** The root parent item from which all the other observations originate. */
		CObservationTreeItem *m_rootitem;

		/** The total number of observations loaded from the rawlog. */
		int m_obs_count = 0;

		/** The unique sensor labels found in the rawlog. */
		std::vector<std::string> m_sensor_labels;

		/** store count of observations of each observation type. */
		std::vector<int> m_count_of_label;

		/** bool to indicate whether the elements in the tree have been synchronized or not. */
		bool m_synced;

		/** m_sync_indices indices of the grouped (synchronized) observations with respect to the original tree, per sensor. */
		std::vector<std::vector<int>> m_sync_indices;

		/** the maximum allowable delay between observation items before they can be synced. */
		int m_sync_offset = -1;

		/** The [R,t] poses of the sensors found in the rawlog. */
		std::vector<Eigen::Matrix4f> m_sensor_poses;

		/** The initial angular and distance uncertainties in the sensor poses. */
		std::vector<Eigen::Vector2f> m_sensor_pose_uncertainties;
};
