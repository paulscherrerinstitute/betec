
A model-3 asynMotor driver to integrate devices from [bestec berlin](https://www.bestec-berlin.de).

Bestec uses EtherCAT bus to integrate motion devices and creates
a TCP/IP socket interface for external access. In addition to the basic control of motor axes,
high level functions like energy/wavelength setting for a monochromator are also provided.

This driver implements the general motion control via a model-3 asynMotor driver, i.e.
*bestecController* and *bestecAxis*. To interface the the devices' high level control, it
is expected to subclass *bestecController* to implement further control parameters.
e.g. *bestecPGM* implments the extra control parameters for a PGM monochromator.
