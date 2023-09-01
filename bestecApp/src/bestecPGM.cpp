
#include <iostream>

#include <epicsString.h>

#include <iocsh.h>

#include <epicsExport.h>

#include "bestec.h"

class epicsShareClass bestecPGMController : public bestecController
{
public:
    bestecPGMController(const char *portName, const char *asynPort, int numAxes,
                     int priority, int stackSize, double movingPollPeriod, double idlePollPeriod);

    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

    virtual asynStatus pollExtra();
    virtual void handleNotification(const char input[], int buflen);

    asynStatus getLimits();

    asynStatus setGratingParameters();
    asynStatus getGratingParameters();

    asynStatus setMonoPose(std::string pose);

private:
    int BestecEnergy;
    int BestecEnergyBusy;
    int BestecEnergyLow;
    int BestecEnergyHigh;
    int BestecGrating;
    int BestecGratingBusy;
    int BestecMTPos;
    int BestecMTPosBusy;
    int BestecLineDensity;
    int BestecDiffOrder;
    int BestecCFF;
};

bestecPGMController::bestecPGMController(const char *portName, const char *asynPort, int numAxes,
                                   int priority, int stackSize, double movingPollPeriod, double idlePollPeriod)
    : bestecController::bestecController(portName, asynPort, numAxes,
                                   priority, stackSize, movingPollPeriod, idlePollPeriod)
{
    createParam("BESTEC_Energy", asynParamFloat64, &BestecEnergy);
    createParam("BESTEC_Energy_BUSY", asynParamInt32, &BestecEnergyBusy);
    createParam("BESTEC_EnergyLow", asynParamFloat64, &BestecEnergyLow);
    createParam("BESTEC_EnergyHigh", asynParamFloat64, &BestecEnergyHigh);

    createParam("BESTEC_Grating", asynParamInt32, &BestecGrating);
    createParam("BESTEC_Grating_BUSY", asynParamInt32, &BestecGratingBusy);

    createParam("BESTEC_MTPos", asynParamFloat64, &BestecMTPos);
    createParam("BESTEC_MTPos_BUSY", asynParamInt32, &BestecMTPosBusy);

    createParam("BESTEC_LINEDENSITY", asynParamFloat64, &BestecLineDensity);
    createParam("BESTEC_DIFFORDER", asynParamInt32, &BestecDiffOrder);
    createParam("BESTEC_CFF", asynParamFloat64, &BestecCFF);

    lock();
    getLimits();
    pollExtra();
    unlock();
}

asynStatus bestecPGMController::getLimits()
{
    double cffLow, cffHigh, energyLow, energyHigh, wavelengthLow, wavelengthHigh, mtLow, mtHigh;
    int  gratingLow, gratingHigh;

    std::string response;
    asynStatus status;

    status = query("LIMITS:M", response);
    if (sscanf(response.c_str(), "CFF %lf %lf Energy %lf %lf Wavelength %lf %lf GratingNr %d %d MTPos %lf %lf",
        &cffLow, &cffHigh, &energyLow, &energyHigh, &wavelengthLow, &wavelengthHigh,
         &gratingLow, &gratingHigh, &mtLow, &mtHigh) == 10) {
        setDoubleParam(BestecEnergyLow, energyLow);
        setDoubleParam(BestecEnergyHigh, energyHigh);
    }
    return status;
}

asynStatus bestecPGMController::setGratingParameters()
{
    double lineDensity, cff;
    int diffOrder;
    char param[MAX_CONTROLLER_STRING_SIZE];
    std::string response;

    getDoubleParam(BestecLineDensity, &lineDensity);
    getIntegerParam(BestecDiffOrder, &diffOrder);
    getDoubleParam(BestecCFF, &cff);

    epicsSnprintf(param, sizeof(param), "%f %d %f", lineDensity, diffOrder, cff);
    return command("SET GRIDPARAMETERS", param, response);
}

asynStatus bestecPGMController::getGratingParameters()
{
    double lineDensity, cff;
    int diffOrder;
    std::string response;
    asynStatus status;

    status = query("GRIDPARAMETERS", response);
    if (sscanf(response.c_str(), "%lf %d %lf", &lineDensity, &diffOrder, &cff) == 3)
    {
        setDoubleParam(BestecLineDensity, lineDensity);
        setIntegerParam(BestecDiffOrder, diffOrder);
        setDoubleParam(BestecCFF, cff);
    }

    return status;
}

asynStatus bestecPGMController::setMonoPose(std::string pose)
{
    int grating, monoState;
    double wavelength, energy, mtPos;

    if (sscanf(pose.c_str(), "%lf %lf %d %lf %d", &wavelength, &energy, &grating, &mtPos, &monoState) == 5) {
        setDoubleParam(BestecEnergy, energy);
        setIntegerParam(BestecGrating, grating);
        setDoubleParam(BestecMTPos, mtPos);
        setIntegerParam(BestecState, monoState);
        return asynSuccess;
    }

    return asynError;
}

asynStatus bestecPGMController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int reason = pasynUser->reason;
    std::string response;
    char param[MAX_CONTROLLER_STRING_SIZE] = {0};
    asynStatus status = asynSuccess;

    if (reason == BestecGrating) {
        epicsSnprintf(param, sizeof(param), "%d FreeSwitch", value);
        status = command("SET GRATINGNR", param, response);
        setIntegerParam(BestecGratingBusy, 1);
    } else if (reason == BestecDiffOrder) {
        setIntegerParam(reason, value);
        status = setGratingParameters();
        getGratingParameters();
    } else {
        status = bestecController::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    return status;
}

asynStatus bestecPGMController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int reason = pasynUser->reason;
    std::string response;
    char param[MAX_CONTROLLER_STRING_SIZE];
    asynStatus status = asynSuccess;

    if (reason == BestecEnergy) {
        epicsSnprintf(param, sizeof(param), "%f", value);
        status = command("SET ENERGY", param, response);
        setIntegerParam(BestecEnergyBusy, 1);
    } else if (reason == BestecMTPos) {
        epicsSnprintf(param, sizeof(param), "%f", value);
        status = command("SET MTPOS", param, response);
        setIntegerParam(BestecMTPosBusy, 1);
    } else if (reason == BestecLineDensity || reason == BestecCFF) {
        setDoubleParam(reason, value);
        status = setGratingParameters();
        getGratingParameters();
    } else {
        status = bestecController::writeFloat64(pasynUser, value);
    }

    callParamCallbacks();
    return status;
}

asynStatus bestecPGMController::pollExtra()
{
    std::string response;
    asynStatus status;
    static const char *functionName = "pollExtra";

    /* Get current grating parameters */
    status = getGratingParameters();
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecPGMController:%s: cannot get grating parameters\n",
                  functionName);
        return status;
    }

    /* Get current mono positions */
    status = query("POSE:M", response);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "bestecPGMController:%s: cannot get mono positions\n",
                  functionName);
        return status;
    }
    status = setMonoPose(response);

    return status;
}

void bestecPGMController::handleNotification(const char input[], int buflen)
{
    int grating;
    double energy, mtPos;

    if (epicsStrnCaseCmp(input, "AUTO ", 5) == 0) {
        if (epicsStrnCaseCmp(input, "AUTO POSE:M ", 12) == 0) {
            setMonoPose(input + 12);
        } else {
            bestecController::handleNotification(input, buflen);
        }
    }
    else if (sscanf(input, "ENERGY:%lf", &energy) == 1) {
        setDoubleParam(BestecEnergy, energy);
        setIntegerParam(BestecEnergyBusy, 0);
    }
    else if (sscanf(input, "MTPOS:%lf", &mtPos) == 1) {
        setDoubleParam(BestecMTPos, mtPos);
        setIntegerParam(BestecMTPosBusy, 0);
    }
    else if (sscanf(input, "GRATINGNR:%d", &grating) == 1) {
        setIntegerParam(BestecGrating, grating);
        setIntegerParam(BestecGratingBusy, 0);
        getGratingParameters();
        getLimits();
    }    
    else {
        bestecController::handleNotification(input, buflen);
    }
}

/** Configuration command, called directly or from iocsh */
extern "C" int bestecPGMCreateController(const char *portName, const char *asynPort, int numAxes, int priority, int stackSize, int movingPollPeriod, int idlePollPeriod)
{
    new bestecPGMController(portName, asynPort, numAxes, priority, stackSize, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg bestecPGMCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg bestecPGMCreateControllerArg1 = {"asyn Port name", iocshArgString};
static const iocshArg bestecPGMCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg bestecPGMCreateControllerArg3 = {"priority", iocshArgInt};
static const iocshArg bestecPGMCreateControllerArg4 = {"stackSize", iocshArgInt};
static const iocshArg bestecPGMCreateControllerArg5 = {"moving poll period [msec]", iocshArgInt};
static const iocshArg bestecPGMCreateControllerArg6 = {"idle poll period [msec]", iocshArgInt};
static const iocshArg *const bestecPGMCreateControllerArgs[] = {&bestecPGMCreateControllerArg0,
                                                             &bestecPGMCreateControllerArg1,
                                                             &bestecPGMCreateControllerArg2,
                                                             &bestecPGMCreateControllerArg3,
                                                             &bestecPGMCreateControllerArg4,
                                                             &bestecPGMCreateControllerArg5,
                                                             &bestecPGMCreateControllerArg6};
static const iocshFuncDef bestecPGMCreateControllerDef = {"bestecPGMCreateController", 7, bestecPGMCreateControllerArgs};
static void bestecPGMCreateControllerCallFunc(const iocshArgBuf *args)
{
    bestecPGMCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

static void bestecPGMDriverRegister(void)
{
    iocshRegister(&bestecPGMCreateControllerDef, bestecPGMCreateControllerCallFunc);
}

extern "C"
{
    epicsExportRegistrar(bestecPGMDriverRegister);
}
