@echo off

SET workspaceFolder=%cd%
SET buildDir=build

mkdir %buildDir%
cd %buildDir%
cmake -G Ninja %workspaceFolder% -DPREFIX=switchingAFC -DSRC_DIR=src/switchingAFC -DSYSTEM_TYPE=Windows -DBUILD_SIMULATION_ONLY=true
ninja
cd %workspaceFolder%