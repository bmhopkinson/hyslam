#ifndef MAPJOB_H_
#define MAPJOB_H_

#include <string>
/*
 * abstract base class for map jobs.
 * defines key functionality:
 * run() - run the map job.
 * abort() - request that the map job stop (not all necessarily will or can)
 * stopped() - informs caller whether the job has stopped based on requested abort.
 * finished()-  informs caller whether the job has finished, that is has completed. the final thing any map job should do
 *    is set has_finished = true
 */

namespace HYSLAM{
class MapJob{
public:
    virtual ~MapJob(){};
    virtual void run() = 0;
    virtual std::string name() = 0;
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