#ifndef MAPJOB_H_
#define MAPJOB_H_

namespace ORB_SLAM2{
class MapJob{
public:
    virtual ~MapJob(){};
    virtual void run() = 0;
    virtual void abort(){abort_requested = true;}
    virtual bool stopped(){return has_stopped;}
    virtual bool finished(){return has_finished;}
protected:
    bool abort_requested = false;
    bool has_stopped = false;
    bool has_finished = false;
};
}

#endif