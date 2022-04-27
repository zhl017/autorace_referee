/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include "../include/rbiz_autorace_monitor/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor
{

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode()
{
    if(ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"rbiz_autorace_monitor");
    if ( ! ros::master::check() )
    {
        return false;
    }
    ros::start();
    ros::NodeHandle n;

    // Publisher
    reset_pub = n.advertise<std_msgs::Bool>("reset", 1);
    state_pub = n.advertise<std_msgs::Int8>("state", 1);
    time_pub = n.advertise<rbiz_autorace_msgs::Time>("time", 1);

    // Subscriber
    vehicle_sub = n.subscribe("vehicle", 1, &QNode::cbReceiveVehicle, this);
    stage_sub = n.subscribe("stage", 1, &QNode::cbReceiveStage, this);

    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(100);
    while ( ros::ok() )
    {
        process();
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// Publish Functions
void QNode::cbReceiveVehicle(const std_msgs::Int8 vehicle_msg)
{
    switch(vehicle_msg.data)
    {
    case 0:
    {
        stopwatchStatus = STAY;
        break;
    }
    case 1:
    {
        stopwatchStatus = READY;
        break;
    }
    case 2:
    {
        stopwatchStatus = START;
        start_trainging_time_ = ros::Time::now();
        break;
    }
    case 3:
    {
        stopwatchStatus = MISSION;
        start_mission_time_ = ros::Time::now();
        break;
    }
    case 4:
    {
        stopwatchStatus = FINISH;
        break;
    }
    }
}

void QNode::cbReceiveStage(const std_msgs::Int8 stage_msg)
{
    switch(stage_msg.data)
    {
    case 0:
    {
        stageStatus = S0;
        break;
    }
    case 1:
    {
        stageStatus = S1;
        start_stage_time_ = ros::Time::now();
        break;
    }
    case 2:
    {
        stageStatus = S1END;
        break;
    }
    case 3:
    {
        stageStatus = S2;
        start_stage_time_ = ros::Time::now();
        break;
    }
    case 4:
    {
        stageStatus = S2END;
        break;
    }
    case 5:
    {
        stageStatus = S3;
        start_stage_time_ = ros::Time::now();
        break;
    }
    case 6:
    {
        stageStatus = S3END;
        break;
    }
    case 7:
    {
        stageStatus = S3FAIL;
        break;
    }
    case 8:
    {
        stageStatus = S4;
        start_stage_time_ = ros::Time::now();
        break;
    }
    }
}

void QNode::pbResetMsg()
{
    std_msgs::Bool reset_msg;
    reset_msg.data = true;
    stopwatchStatus = STAY;
    reset_pub.publish(reset_msg);

    rbiz_autorace_msgs::Time reset_time_msg;
    time_msg = reset_time_msg;
    time_pub.publish(time_msg);
}

void QNode::pbStateMsg(int state)
{
    std_msgs::Int8 state_msg;
    state_msg.data = state;
    state_pub.publish(state_msg);
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	// logging_model.insertRows(logging_model.rowCount(),1);
	// std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				// logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				// logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				// logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				// logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				// logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	// QVariant new_row(QString(logging_model_msg.str().c_str()));
	// logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	// Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::process()
{
    switch(stopwatchStatus)
    {
        case STAY:
        {
            start_trainging_time_ = ros::Time::now();
            Q_EMIT resetTrainingTime();
            break;
        }
        case READY:
        {
            ros::Time current_time = ros::Time::now();

            ros::Duration dur_1 = current_time - start_trainging_time_;
            ros::Duration dur = ros::Duration(5 * 60) - dur_1;
            int min = dur.sec / 60;
            int sec = dur.sec % 60;
            int m_sec = dur.nsec / 10e6;

            Q_EMIT setTrainingTime(min, sec, m_sec);

            if (dur <= ros::Duration(0))
            {
                stopwatchStatus = START;
                pbStateMsg(1);
            }

            break;
        }

        case START:
        {
            Q_EMIT readyMissionTime();
            break;
        }
        case MISSION:
        {
            ros::Time mission_time = ros::Time::now();
            ros::Duration dur = mission_time - start_mission_time_;

            int min = dur.sec / 60;
            int sec = dur.sec % 60;
            int m_sec = dur.nsec / 10e6;

            time_msg.mission_time[0] = min;
            time_msg.mission_time[1] = sec;
            time_msg.mission_time[2] = m_sec;

            Q_EMIT startMissionTime(min, sec, m_sec);

            // switch(stageStatus)
            // {
            //     case S1:
            //     {
            //         ros::Time stage_time = ros::Time::now();
            //         ros::Duration dur = stage_time - start_stage_time_;
            //         int min = dur.sec / 60;
            //         int sec = dur.sec % 60;
            //         int m_sec = dur.nsec / 10e6;
            //         time_msg.stage1_time[0] = min;
            //         time_msg.stage1_time[1] = sec;
            //         time_msg.stage1_time[2] = m_sec;
            //         Q_EMIT startStage1Time(min, sec, m_sec);
            //         break;
            //     }

            //     case S1END:
            //     {
            //         Q_EMIT finishStage1();
            //         break;
            //     }

            //     case S2:
            //     {
            //         ros::Time stage_time = ros::Time::now();
            //         ros::Duration dur = stage_time - start_stage_time_;
            //         int min = dur.sec / 60;
            //         int sec = dur.sec % 60;
            //         int m_sec = dur.nsec / 10e6;
            //         time_msg.stage2_time[0] = min;
            //         time_msg.stage2_time[1] = sec;
            //         time_msg.stage2_time[2] = m_sec;
            //         Q_EMIT startStage2Time(min, sec, m_sec);
            //         break;
            //     }


            //     case S2END:
            //     {
            //         Q_EMIT finishStage2();
            //         break;
            //     }


            //     case S3:
            //     {
            //         Q_EMIT finishStage2();
            //         ros::Time stage_time = ros::Time::now();
            //         ros::Duration dur = stage_time - start_stage_time_;
            //         int min = dur.sec / 60;
            //         int sec = dur.sec % 60;
            //         int m_sec = dur.nsec / 10e6;
            //         time_msg.stage3_time[0] = min;
            //         time_msg.stage3_time[1] = sec;
            //         time_msg.stage3_time[2] = m_sec;
            //         Q_EMIT startStage3Time(min, sec, m_sec);
            //         break;
            //     }

            //     case S3END:
            //     {
            //         Q_EMIT finishStage3();
            //         break;
            //     }

            //     case S3FAIL:
            //     {
            //         Q_EMIT failStage3();
            //         break;
            //     }

            //     case S4:
            //     {
            //         ros::Time stage_time = ros::Time::now();
            //         ros::Duration dur = stage_time - start_stage_time_;
            //         int min = dur.sec / 60;
            //         int sec = dur.sec % 60;
            //         int m_sec = dur.nsec / 10e6;
            //         time_msg.stage4_time[0] = min;
            //         time_msg.stage4_time[1] = sec;
            //         time_msg.stage4_time[2] = m_sec;
            //         Q_EMIT startStage4Time(min, sec, m_sec);
            //         break;
            //     }
            //     default:
            //         break;
            // }

            time_pub.publish(time_msg);

            if (dur.sec >= 5 * 60)     // time out 5 min
            {
                stopwatchStatus = TIMEOUT;
                pbStateMsg(2);
            }
            break;
        }
        case FINISH:
            Q_EMIT finishMission();
            // Q_EMIT finishStage4();
            break;
        case TIMEOUT:
            Q_EMIT timeOut();
            break;
        default:
            break;
    }
}
} // namespace rbiz_autorace_monitor
