/**
 * @file /include/rbiz_autorace_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rbiz_autorace_monitor_QNODE_HPP_
#define rbiz_autorace_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <rbiz_autorace_msgs/Time.h>
#include <string>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor
{
/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
    Q_OBJECT
public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        bool init();
        bool init(const std::string &master_url, const std::string &host_url);
        void run();

        double lap_time = 0.0;
        double mission_time = 0.0;

        enum StopwatchStatus {
            STAY,
            READY,
            START,
            MISSION,
            FINISH,
            TIMEOUT,
        } stopwatchStatus;

        enum Stagestatus {
            S0,
            S1,
            S1END,
            S2,
            S2END,
            S3,
            S3END,
            S3FAIL,
            S4,
        } stageStatus;

        enum LogLevel {
            Debug,
            Info,
            Warn,
            Error,
            Fatal
         };

        // Publisher function
        void pbResetMsg();
        void pbStateMsg(int state);

        // Subscriber function
        void cbReceiveVehicle(const std_msgs::Int8 vehicle_msg);
        void cbReceiveStage(const std_msgs::Int8 stage_msg);
        void cbReceiveTime(const rbiz_autorace_msgs::Time time_msg);

        // Normal function
        void log( const LogLevel &level, const std::string &msg);


      Q_SIGNALS:
        void resetTrainingTime();
        void startTrainingTime();
        void stopTrainingTime();
        void readyAllTime();
        void setTrainingTime(int min, int sec, int m_sec);
        void readyMissionTime();
        void startMissionTime(int min, int sec, int m_sec);
        void finishMission();
        void timeOut();
        void rosShutdown();

        void startStage1Time(int min, int sec, int m_sec);
        void finishStage1();
        void startStage2Time(int min, int sec, int m_sec);
        void finishStage2();
        void startStage3Time(int min, int sec, int m_sec);
        void finishStage3();
        void failStage3();
        void startStage4Time(int min, int sec, int m_sec);
        void finishStage4();
        void resetStage();

private:
        int init_argc;
        char** init_argv;

        void process();

        // Publisher
        ros::Publisher reset_pub;
        ros::Publisher state_pub;
        ros::Publisher time_pub;

        // Subscriber
        ros::Subscriber stage_sub;
        ros::Subscriber vehicle_sub;

        // Messages
        ros::Time start_trainging_time_;
        ros::Time start_mission_time_;
        ros::Time start_stage_time_;

        rbiz_autorace_msgs::Time time_msg;
};
}  // namespace rbiz_autorace_monitor
#endif /* rbiz_autorace_monitor_QNODE_HPP_ */
