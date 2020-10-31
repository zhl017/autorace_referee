/**
 * @file /include/rbiz_autorace_monitor/main_window.hpp
 *
 * @brief Qt based gui for rbiz_autorace_monitor.
 *
 * @date November 2010
 **/
#ifndef rbiz_autorace_monitor_MAIN_WINDOW_H
#define rbiz_autorace_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rbiz_autorace_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	// void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

        //void setreset();

        void on_pushButton_clicked(bool check);
	// void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
        void readyAllTime();
        void startReadyTime();
        void resetTrainingTime();
        void setTrainingTime(int min, int sec, int m_sec);
        void readyMissionTime();
        void startMissionTime(int min, int sec, int m_sec);
        void finishMission();
        void timeOut();
    // void updateLoggingView(); // no idea why this can't connect automatically
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
	Ui::MainWindowDesign ui;
	QNode qnode;

        ros::Time start_ready_time_;
        ros::Time start_play_time_;
};

}  // namespace rbiz_autorace_monitor

#endif // rbiz_autorace_monitor_MAIN_WINDOW_H
