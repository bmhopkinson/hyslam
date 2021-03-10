//
// Created by cv-bhlab on 3/9/21.
//

#ifndef HYSLAM_INTERTHREAD_H
#define HYSLAM_INTERTHREAD_H

#include <mutex>
#include <FeatureViews.h>
#include <SensorData.h>
#include <ORBSLAM_datastructs.h>

namespace HYSLAM {


struct ImageFeatureData{
    FeatureViews LMviews;
    cv::Mat image;
    Imgdata img_data;
    SensorData sensor_data;
};


struct ThreadStatus {

    void clearPostStop();

    bool isStopped() const;

    bool isStopRequested() const;

    bool isStoppable() const;

    bool isRelease() const;

    bool isInterrupt() const;

    bool isFinished() const;

    bool isFinishRequested() const;

    bool isAcceptingInput() const;

    void setIsStopped(bool isStopped);

    void setStopRequested(bool stopRequested);

    void setStoppable(bool stoppable);

    void setRelease(bool release);

    void setInterrupt(bool interrupt);

    void setIsFinished(bool isFinished);

    void setFinishRequested(bool finishRequested);

    void setAcceptingInput(bool acceptingInput);

    bool is_stopped = false;
    bool stop_requested = false;
    bool stoppable = true;
    bool release = false;
    bool interrupt = false;
    int queue_length = 0;

    int getQueueLength() const;

    void setQueueLength(int queueLength);

    bool is_finished = false;
    bool finish_requested = false;
    bool accepting_input = true;

    mutable std::mutex mutex_ts;
};

struct MainThreadsStatus {
    ThreadStatus tracking;
    ThreadStatus mapping;

};

}

#endif //HYSLAM_INTERTHREAD_H
