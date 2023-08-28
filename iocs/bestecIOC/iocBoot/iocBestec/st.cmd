#!../../bin/RHEL7-x86_64/bestec

#- You may have to change bestec to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/bestec.dbd"
bestec_registerRecordDeviceDriver pdbbase


drvAsynIPPortConfigure("BS_IP", "127.0.0.1:45401")
bestecPGMCreateController("BS", "BS_IP", 4, 0, 0, 100, 100)


cd "${TOP}/iocBoot/${IOC}"

dbLoadTemplate("bestecPGM.subs")

iocInit
