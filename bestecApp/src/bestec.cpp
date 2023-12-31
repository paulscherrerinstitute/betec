#include <string.h>
#include <stdlib.h>

#include <string>

#include <epicsAlgorithm.h>
#include <epicsString.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>

#include "bestec.h"

bestecController::bestecController(const char *portName, const char *asynPort, int numAxes,
                                   int priority, int stackSize, double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, 0,
                          0, // No additional interfaces beyond those in base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,                   // autoconnect
                          priority, stackSize), // Default priority and stack size
    serverIsConnected_(true)
{
    asynStatus status;
    static const char *functionName = "bestecController";

    /* Create controller specific parameters */
    createParam("BESTEC_STOP", asynParamInt32, &BestecStop);
    createParam("BESTEC_STATE", asynParamInt32, &BestecState);
    createParam("BESTEC_MSG", asynParamOctet, &BestecMsg);

    createParam("BESTEC_STABSTATE", asynParamInt32, &BestecStabState);

    createParam("BESTEC_LIMENC_ERROR", asynParamInt32, &BestecLimitEncoderError);
    createParam("BESTEC_MOTION_ERROR", asynParamInt32, &BestecMotionError);
    createParam("BESTEC_CLEAR_FL_ERROR", asynParamInt32, &BestecClearFollowingError);
    createParam("BESTEC_MOTOR_AUX1", asynParamInt32, &BestecMotorAux1);
    createParam("BESTEC_MOTOR_AUX2", asynParamInt32, &BestecMotorAux2);

    /* Message queues for notification messages */
    notifyQ_ = epicsMessageQueueCreate(100, MAX_CONTROLLER_STRING_SIZE);

    /* Connect to BESTEC ProcessServer */
    status = pasynOctetSyncIO->connect(asynPort, 0, &pasynUserController_, NULL);
    if (status)
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController::%s: cannot connect to bestec ProcessServer\n",
                  functionName);
    pasynOctetSyncIO->setInputEos(pasynUserController_, "\r\n", 2);
    pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);

    /* Create axes */
    for (int axis = 0; axis < numAxes_; axis++)
        pAxes_[axis] = new bestecAxis(this, axis);

    /* Connect to BESTEC ProcessServer */
    connectServer();

    /* Poll notification messages */
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

asynStatus bestecController::connectServer()
{
    char param[MAX_CONTROLLER_STRING_SIZE];
    std::string response;
    asynStatus status = asynSuccess;
    static const char *functionName = "connectServer";

    /* Login as Expert user */
    status = command("USER LOG IN", "Expert Expert", response);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: cannot login as Expert:Expert\n",
                  functionName);
        goto done;
    }

    /* Get axis parameters and position */
    for (int axis=0; axis<numAxes_; axis++) {
        int scale = 1;
        bestecAxis *pAxis = getAxis(axis);
        epicsSnprintf(param, sizeof(param), "AXISPARAMS:%d", axis+1);
        if (!query(param, response) &&
            sscanf(response.c_str(), "%*d %*d %d %*s", &scale) == 1) {
            pAxis->setAxisScale(scale);
            pAxis->setDoubleParam(motorRecResolution_, 1.0 / scale);
        }
        epicsSnprintf(param, sizeof(param), "AXISSTATE:%d", axis+1);
        if (!query(param, response)) {
            getAxis(axis)->setAxisState(response);
        }
    }

    status = query("STABSTATE", response);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: cannot get stabilization state\n",
                  functionName);
        goto done;
    }
    setIntegerParam(BestecStabState, atoi(response.c_str()));

    status = pollExtra();

    /* Enable notification */
    status = command("ENABLE NOTIFY", "1", response);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: cannot enable notification\n",
                  functionName);
        goto done;
    }

done:
    if (status == asynSuccess) {
        serverIsConnected_ = true;
    } else {
        serverIsConnected_ = false;
        for (int i=0; i<numAxes_; i++) {
            bestecAxis *pAxis = getAxis(i);
            pAxis->setAxisState("");
            disconnect(pAxis->pasynUser_);
        }
    }

    return status;
}

asynStatus bestecController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int reason = pasynUser->reason;
    std::string response;
    char param[MAX_CONTROLLER_STRING_SIZE] = {0};
    asynStatus status = asynSuccess;

    if (reason == BestecStop) {
        status = command("SYSTEM STOP", "", response);
    } else if (reason == BestecStabState)
    {
        epicsSnprintf(param, sizeof(param), "%d", value);
        status = command("SET STABSTATE", param, response);
        if (query("STABSTATE", response) == asynSuccess)
            setIntegerParam(reason, atoi(response.c_str()));
    } else if (reason == BestecClearFollowingError) {
        status = command("CLEAR FOLLOWINGERROR", param, response);
    } else {
        status = asynMotorController::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    return status;
}

void bestecController::report(FILE *fp, int level)
{
    fprintf(fp, "bestecController %s\n", this->portName);
    fprintf(fp, "    numAxes: %d\n", numAxes_);
    fprintf(fp, "    moving poll period: %f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period: %f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

asynStatus bestecController::connect(asynUser *pasynUser)
{
    int addr = 0;
    asynStatus status;
    static const char *functionName = "connect";

    status = getAddress(pasynUser, &addr);
    if (status) return status;

    if (!serverIsConnected_) {
        /* Only try connection for addr = 0 */
        if (addr == 0) {
            status = connectServer();
            if (status != asynSuccess) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "bestecController:%s: server connection failed\n",
                    functionName);
                return status;
            }
        } else {
            return asynDisconnected;
        }
    }

    return asynPortDriver::connect(pasynUser);
}

bestecAxis *bestecController::getAxis(int axisNo)
{
    return dynamic_cast<bestecAxis *>(asynMotorController::getAxis(axisNo));
}

static void bestecPollerC(void *drvPvt)
{
    bestecController *pController = (bestecController *)drvPvt;
    pController->poller();
}

asynStatus bestecController::startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls)
{
    movingPollPeriod_ = movingPollPeriod;
    idlePollPeriod_ = idlePollPeriod;
    forcedFastPolls_ = forcedFastPolls;
    epicsThreadCreate("bestecPoller",
                      epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)bestecPollerC, (void *)this);
    return asynSuccess;
}

void bestecController::handleNotification(const char input[], int buflen)
{
    int axisNo;
    int globalMotion, globalStabilization, globalLimitEncoderError, globalMotionError, globalMotionErrorDesc;
    int pos;
    static const char *functionName = "handleNotification";

    if (epicsStrnCaseCmp(input, "AUTO ", 5) == 0) {
        if (sscanf(input, "AUTO STATE:%d %d %d %d %d",
                        &globalMotion, &globalStabilization, &globalLimitEncoderError,
                        &globalMotionError, &globalMotionErrorDesc) == 5) {
            setIntegerParam(BestecState, globalMotion);
            setIntegerParam(BestecStabState, globalStabilization);
            setIntegerParam(BestecLimitEncoderError, globalLimitEncoderError);
            setIntegerParam(BestecMotionError, abs(globalMotionError));
            setIntegerParam(motorStatusFollowingError_, globalMotionError == -7);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "bestecController:%s: Unknown notification`%s`\n",
                      functionName, input);
        }
    } else if (epicsStrnCaseCmp(input, "ERROR: ", 7) == 0) {
        setStringParam(BestecMsg, input + 7);
    } else if (epicsStrnCaseCmp(input, "CONNECTION ESTABLISHED, ", 24) == 0) {
        /* The greeting message is ignored */
    } else if (sscanf(input, "AXISSTATE:%d%n", &axisNo, &pos) == 1) {
        getAxis(axisNo-1)->setAxisState(input + pos);
    } else if (sscanf(input, "MOTION FINISHED Axis-%d", &axisNo) == 1) {
        /* Extra confirmation message that axis has finished movement */
    } else if (sscanf(input, "AXIS MOTION FINISHED:%d", &axisNo) == 1 ||
        sscanf(input, "AXIS MOTION STOPPED:%d", &axisNo) == 1) {
        getAxis(axisNo-1)->setIntegerParam(motorStatusDone_, 1);
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "bestecController:%s: Unknown notification `%s`\n",
            functionName, input);
    }
}

void bestecController::poller()
{
    char input[MAX_CONTROLLER_STRING_SIZE] = {0};
    size_t nread;
    int eomReason = 0;
    epicsTimeStamp start, now;
    asynStatus status = asynSuccess;
    static const char *functionName = "poller";

    while (1)
    {
        epicsTimeGetCurrent(&start);
        lock();

        /* Poll messages from the server */
        status = pasynOctetSyncIO->read(pasynUserController_, input, sizeof(input), 0, &nread, &eomReason);

        if (status == asynSuccess) {
            if (eomReason == ASYN_EOM_EOS)
                handleNotification(input, sizeof(input));
        } else if (status == asynTimeout) {
            /* Poll messages from the queue */
            if (epicsMessageQueueTryReceive(notifyQ_, input, sizeof(input)) > 0)
                handleNotification(input, sizeof(input));
        } else if (status == asynDisconnected) {
            if (serverIsConnected_) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "bestecController:%s: disconnected from the server\n",
                      functionName);
                serverIsConnected_ = false;
                for (int i=0; i<numAxes_; i++) {
                    bestecAxis *pAxis = getAxis(i);
                    pAxis->setAxisState("");
                    disconnect(pAxis->pasynUser_);
                }
            }
        }

        unlock();
        epicsTimeGetCurrent(&now);

        /* The server pushes 3 notifications every 100 ms.
           We limit the polling loop to 2ms, and give other port functions time to run.
         */
        double dt = 0.002 - epicsTimeDiffInSeconds(&now, &start);
        if (dt > 0) {
            epicsThreadSleep(dt);
        }
    }
}

asynStatus bestecController::query(const std::string param, std::string &response)
{
    char out[MAX_CONTROLLER_STRING_SIZE] = {0};
    char resp[MAX_CONTROLLER_STRING_SIZE] = {0};
    char confirm[MAX_CONTROLLER_STRING_SIZE] = {0};
    char reject[MAX_CONTROLLER_STRING_SIZE] = {0};
    epicsTimeStamp start, now;
    int confirm_len, reject_len;
    size_t pos;
    asynStatus status = asynSuccess;
    const static char *functionName = "query";

    /* Query buffer, GET <param> */
    epicsSnprintf(out, sizeof(out), "GET %s", param.c_str());

    /* Pattern of the reject reply */
    if ((pos = param.find(':')) != std::string::npos)
        reject_len = epicsSnprintf(reject, sizeof(reject), "GET %s:n", param.substr(0, pos).c_str());
    else
        reject_len = epicsSnprintf(reject, sizeof(reject), "GET %s:n", param.c_str());

    /* Pattern of the confirmation reply */
    if (param.find(':') != std::string::npos)
        confirm_len = epicsSnprintf(confirm, sizeof(confirm), "%s ", param.c_str());
    else
        confirm_len = epicsSnprintf(confirm, sizeof(confirm), "%s:", param.c_str());

    epicsTimeGetCurrent(&start);

    /* Send query */
    status = writeController(out, 0);
    if (status) return status;

    /* Wait for reply */
    while (1) {
        size_t nread = 0;
        int eomReason = 0;
        status = pasynOctetSyncIO->read(pasynUserController_, resp, sizeof(resp), 0, &nread, &eomReason);

        if (status == asynSuccess && nread > 0) {
            if (epicsStrnCaseCmp(resp, confirm, confirm_len) == 0) {
                response = resp + confirm_len;
                break;
            } else if (epicsStrnCaseCmp(resp, reject, reject_len) == 0) {
                status = asynError;
                setStringParam(BestecMsg, resp);
                response = resp + reject_len;
                break;
            } else {
                epicsMessageQueueSend(notifyQ_, resp, sizeof(resp));
            }
        }

        epicsTimeGetCurrent(&now);
        if (epicsTimeDiffInSeconds(&now, &start) > DEFAULT_CONTROLLER_TIMEOUT) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: '%s' timeout\n",
                  functionName, out);
            status = asynTimeout;
            break;
        }
    }
    return status;
}

asynStatus bestecController::command(const std::string command, const std::string parameter, std::string &response)
{
    char out[MAX_CONTROLLER_STRING_SIZE] = {0};
    char resp[MAX_CONTROLLER_STRING_SIZE] = {0};
    char confirm[MAX_CONTROLLER_STRING_SIZE] = {0};
    epicsTimeStamp start, now;
    int len;
    asynStatus status;
    const static char *functionName = "command";

    /* Command buffer, <command>[:<parameter>] */
    if (parameter.empty())
        epicsSnprintf(out, sizeof(out), "%s", command.c_str());
    else
        epicsSnprintf(out, sizeof(out), "%s:%s", command.c_str(), parameter.c_str());

    /* Pattern of the expected reply */
    if (epicsStrCaseCmp(command.c_str(), "SYSTEM STOP") == 0)
        len = epicsSnprintf(confirm, sizeof(confirm), "SYSTEM IS STOPPED");
    else
        len = epicsSnprintf(confirm, sizeof(confirm), "%s", command.c_str());

    epicsTimeGetCurrent(&start);

    /* Send command */
    status = writeController(out, 0);
    if (status) return status;

    /* Wait for reply */
    while (1) {
        size_t nread = 0;
        int eomReason = 0;
        status = pasynOctetSyncIO->read(pasynUserController_, resp, sizeof(resp), 0, &nread, &eomReason);

        if (status == asynSuccess && nread > 0) {
            if (epicsStrnCaseCmp(resp, confirm, len) == 0)
                break;
            else
                epicsMessageQueueSend(notifyQ_, resp, sizeof(resp));
        }

        /* Check for timeout */
        epicsTimeGetCurrent(&now);
        if (epicsTimeDiffInSeconds(&now, &start) > DEFAULT_CONTROLLER_TIMEOUT) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: '%s' timeout\n",
                  functionName, out);
            status = asynTimeout;
            break;
        }
    }

    if (status) return status;

    if (epicsStrCaseCmp(command.c_str(), "SYSTEM STOP") == 0)
        return asynSuccess;

    /* Check whether the command is accepted */
    len = epicsSnprintf(confirm, sizeof(confirm), "%s:y", command.c_str());

    if (epicsStrnCaseCmp(resp, confirm, len) == 0)
    {
        if (resp[len] == ' ')
            len += 1;
        response = resp + len;
        return asynSuccess;
    }

    // Check error message if the command is rejected
    len = epicsSnprintf(confirm, sizeof(confirm), "%s:n ", command.c_str());

    if (epicsStrnCaseCmp(resp, confirm, len) == 0)
    {
        response = resp + len;
        const char * error = resp + len;
        setStringParam(BestecMsg, resp);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecController:%s: '%s' rejected: %s\n",
                  functionName, out, error);
        return asynError;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "bestecController:%s: '%s' unexpected reply: %s\n",
        functionName, out, resp);
    return asynError;
}

bestecAxis::bestecAxis(bestecController *pC, int axisNum)
    : asynMotorAxis(pC, axisNum), pC_(pC), scale_(1), motionInProgress_(false)
{
}

asynStatus bestecAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    char command[MAX_CONTROLLER_STRING_SIZE], param[MAX_CONTROLLER_STRING_SIZE];
    double resolution, velocity;
    std::string response;
    asynStatus status;
    static const char *functionName = "move";

    pC_->getDoubleParam (axisNo_, pC_->motorRecResolution_, &resolution);

    // Assume velocity is in the range of [0.01, 1.0]. 1.0 means 100% velocity.
    velocity = epicsMin(1.0, epicsMax(0.01, maxVelocity * resolution));

    if (relative)
        sprintf(command, "AXIS MOVEUNITSDELTA");
    else
        sprintf(command, "AXIS MOVEUNITSABS");

    epicsSnprintf(param, sizeof(param), "%d %f %f", axisNo_+1, position * resolution, velocity);

    // Flag a motion started by this diver
    motionInProgress_ = true;

    status = pC_->command(command, param, response);
    if (status == asynError)
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecAxis:%s: error '%s'\n",
                  functionName, response.c_str());

    return status;
}

asynStatus bestecAxis::setAxisState(std::string state)
{
    int axisSteps, moveFlag, lowLimitSwitch, highLimitSwitch, stabilized, aux1, aux2;
    double resolution, axisPosition;
    asynStatus status = asynSuccess;

    if (sscanf(state.c_str(), "%d %lf %d %d %d %d %d %d",
               &axisSteps, &axisPosition, &moveFlag,
               &lowLimitSwitch, &highLimitSwitch,
               &stabilized, &aux1, &aux2) == 8) {

        pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);

        /* The axis state is from the server's periodic notification. It can happen
           that driver has started a motion, but an older notification message tells
           the axis is(actually was) not moving. This can lead to prematurely declare a motion finished.

           So this moveFlag is used for motions not started by this driver.
           */
        if (!motionInProgress_)
            setIntegerParam(pC_->motorStatusDone_, !moveFlag);

        setDoubleParam(pC_->motorEncoderPosition_, axisPosition / resolution);
        setDoubleParam(pC_->motorPosition_, axisPosition / resolution);

        setIntegerParam(pC_->motorStatusLowLimit_, lowLimitSwitch);
        setIntegerParam(pC_->motorStatusHighLimit_, highLimitSwitch);

        setIntegerParam(pC_->BestecMotorAux1, aux1);
        setIntegerParam(pC_->BestecMotorAux2, aux2);

        // update motor status if polling was successful
        setIntegerParam(pC_->motorStatusCommsError_, 0);
        setIntegerParam(pC_->motorStatusProblem_, 0);
    } else {
        status = asynError;
        setIntegerParam(pC_->motorStatusCommsError_, 1);
        setIntegerParam(pC_->motorStatusProblem_, 1);
        statusChanged_ = 1;
    }

    callParamCallbacks();
    return status;
}

asynStatus bestecAxis::stop(double acceleration)
{
    char param[MAX_CONTROLLER_STRING_SIZE];
    std::string response;
    asynStatus status;
    static const char *functionName = "stop";

    epicsSnprintf(param, sizeof(param), "%d", axisNo_+1);
    status = pC_->command("AXIS STOP", param, response);
    if (status == asynError)
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecAxis:%s: error '%s'\n",
                  functionName, response.c_str());

    return status;
}

void bestecAxis::report(FILE *fp, int level)
{
    fprintf(fp, "bestecAxis: %d\n", axisNo_);
    fprintf(fp, "    scale: %d\n", scale_);

    // Call the base class method
    asynMotorAxis::report(fp, level);
}

/** Configuration command, called directly or from iocsh */
extern "C" int bestecCreateController(const char *portName, const char *asynPort, int numAxes, int priority, int stackSize, int movingPollPeriod, int idlePollPeriod)
{
    new bestecController(portName, asynPort, numAxes, priority, stackSize, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg bestecCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg bestecCreateControllerArg1 = {"asyn Port name", iocshArgString};
static const iocshArg bestecCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg bestecCreateControllerArg3 = {"priority", iocshArgInt};
static const iocshArg bestecCreateControllerArg4 = {"stackSize", iocshArgInt};
static const iocshArg bestecCreateControllerArg5 = {"moving poll period [msec]", iocshArgInt};
static const iocshArg bestecCreateControllerArg6 = {"idle poll period [msec]", iocshArgInt};
static const iocshArg *const bestecCreateControllerArgs[] = {&bestecCreateControllerArg0,
                                                             &bestecCreateControllerArg1,
                                                             &bestecCreateControllerArg2,
                                                             &bestecCreateControllerArg3,
                                                             &bestecCreateControllerArg4,
                                                             &bestecCreateControllerArg5,
                                                             &bestecCreateControllerArg6};
static const iocshFuncDef bestecCreateControllerDef = {"bestecCreateController", 7, bestecCreateControllerArgs};
static void bestecCreateControllerCallFunc(const iocshArgBuf *args)
{
    bestecCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

static void bestecDriverRegister(void)
{
    iocshRegister(&bestecCreateControllerDef, bestecCreateControllerCallFunc);
}

extern "C"
{
    epicsExportRegistrar(bestecDriverRegister);
}
