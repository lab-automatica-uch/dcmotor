Planta para control de motor CC
===============================

Para compilar en Windows usar:

```
mex -lWSock32 -Iinclude src/SDCMotor.cpp src/opto22snap.cpp
```

En Linux

```
mex -D_LINUX -Iinclude src/SDCMotor.cpp src/opto22snap.cpp
```
