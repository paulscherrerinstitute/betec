include /ioc/tools/driver.makefile
PROJECT = bestec
EXCLUDE_VERSIONS += 3
BUILDCLASSES += Linux

SOURCES += ../bestecApp/src/bestec.cpp
SOURCES += ../bestecApp/src/bestecPGM.cpp
DBDS += ../bestecApp/src/bestecSupport.dbd

TEMPLATES += ../bestecApp/Db/bestecAxisExtra.template
TEMPLATES += ../bestecApp/Db/bestecController.template
TEMPLATES += ../bestecApp/Db/bestecPGM.template
TEMPLATES += ../bestecApp/Db/bestecResetBusy.template

QT += ../bestecApp/op/ui/bestecMono.ui
