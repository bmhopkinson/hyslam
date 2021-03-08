#include <ORBSLAM_datastructs.h>

namespace HYSLAM {

Imgdata::Imgdata(std::string name_, std::string camera_, double time_stamp_) :
          name(name_), camera(camera_), time_stamp(time_stamp_){}

void ThreadStatus::clearPostStop(){
    std::lock_guard<std::mutex> lock(mutex_ts);
    is_stopped = false;
    stop_requested = false;
    release = false;

}

    bool ThreadStatus::isRelease() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return release;
    }

    bool ThreadStatus::isFinished() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return is_finished;
    }

    bool ThreadStatus::isFinishRequested() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return finish_requested;
    }

    bool ThreadStatus::isAcceptingInput() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return accepting_input;
    }

    bool ThreadStatus::isStopped() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return is_stopped;
    }

    bool ThreadStatus::isStopRequested() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return stop_requested;
    }

    bool ThreadStatus::isStoppable() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return stoppable;
    }

    void ThreadStatus::setIsStopped(bool isStopped) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        is_stopped = isStopped;
    }

    void ThreadStatus::setStopRequested(bool stopRequested) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        stop_requested = stopRequested;
    }

    void ThreadStatus::setStoppable(bool stoppable_) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        stoppable = stoppable_;
    }

    void ThreadStatus::setRelease(bool release_) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        release = release_;
    }

    void ThreadStatus::setIsFinished(bool isFinished) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        is_finished = isFinished;
    }

    void ThreadStatus::setFinishRequested(bool finishRequested) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        finish_requested = finishRequested;
    }

    void ThreadStatus::setAcceptingInput(bool acceptingInput) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        accepting_input = acceptingInput;
    }

    bool ThreadStatus::isInterrupt() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return interrupt;
    }

    void ThreadStatus::setInterrupt(bool interrupt) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        ThreadStatus::interrupt = interrupt;
    }

    int ThreadStatus::getQueueLength() const {
        std::lock_guard<std::mutex> lock(mutex_ts);
        return queue_length;
    }

    void ThreadStatus::setQueueLength(int queueLength) {
        std::lock_guard<std::mutex> lock(mutex_ts);
        queue_length = queueLength;
    }

    ORBExtractorParams::ORBExtractorParams(FeatureExtractor* extractor){
  setParams(extractor);
}

void ORBExtractorParams::setParams(FeatureExtractor* extractor){
    mnScaleLevels = extractor->GetLevels();
    mfScaleFactor = extractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = extractor->GetScaleFactors();
    mvInvScaleFactors = extractor->GetInverseScaleFactors();
    mvLevelSigma2 = extractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = extractor->GetInverseScaleSigmaSquares();
}

} //close namespace
