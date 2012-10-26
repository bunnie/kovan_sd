@echo off

rem  PlanAhead(TM)
rem  runme.bat: a PlanAhead-generated ExploreAhead Script
rem  Copyright 1986-1999, 2001-2011 Xilinx, Inc. All Rights Reserved.


set HD_SDIR=%~dp0
cd /d "%HD_SDIR%"
cscript /nologo /E:JScript "%HD_SDIR%\rundef.js" %*
