#ifndef BESTEC_H
#define BESTEC_H

#include <epicsMessageQueue.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

class bestecController;

class epicsShareClass bestecAxis : public asynMotorAxis
{
public:
    bestecAxis(bestecController *pC, int axisNum);
    virtual asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
    virtual asynStatus stop(double acceleration);
    virtual void report(FILE *fp, int level);

    asynStatus setAxisState(std::string state);
    void setAxisScale(int scale) {scale_ = scale;}

protected:
    bestecController *pC_;
    int scale_;
    bool motionInProgress_;

    friend class bestecController;
};

class epicsShareClass bestecController : public asynMotorController
{
public:
    bestecController(const char *portName, const char *asynPort, int numAxes,
                     int priority, int stackSize, double movingPollPeriod, double idlePollPeriod);

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void report(FILE *fp, int level);
    virtual asynStatus connect(asynUser *pasynUser);

    virtual bestecAxis *getAxis(int axisNo);
    virtual asynStatus startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls);
    void poller();

    asynStatus connectServer();
    asynStatus command(std::string command, std::string parameter, std::string &response);
    asynStatus query(const std::string query, std::string &response);

    virtual asynStatus pollExtra() {return asynSuccess;}
    virtual void handleNotification(const char input[], int buflen);

protected:
    int BestecStop;
    int BestecState;
    int BestecMsg;
    int BestecStabState;
    int BestecLimitEncoderError;
    int BestecMotionError;
    int BestecClearFollowingError;
    int BestecMotorAux1;
    int BestecMotorAux2;

    bool serverIsConnected_;
    epicsMutexId socketLock_;
    epicsMessageQueueId notifyQ_;

    friend class bestecAxis;
};

#endif
