#!/bin/sh

# 
# PlanAhead(TM)
# runme.sh: a PlanAhead-generated ExploreAhead Script for UNIX
# Copyright 1986-1999, 2001-2011 Xilinx, Inc. All Rights Reserved.
# 

echo "This script was generated under a different operating system."
echo "Please update the PATH and LD_LIBRARY_PATH variables below, before executing this script"
exit

if [ -z "$PATH" ]; then
  PATH=C:/Xilinx/13.3/ISE_DS/ISE/bin/nt64;C:/Xilinx/13.3/ISE_DS/ISE/lib/nt64;C:/Xilinx/13.3/ISE_DS/common/bin/nt64;C:/Xilinx/13.3/ISE_DS/common/lib/nt64:C:/Xilinx/13.3/ISE_DS/PlanAhead\\bin
else
  PATH=C:/Xilinx/13.3/ISE_DS/ISE/bin/nt64;C:/Xilinx/13.3/ISE_DS/ISE/lib/nt64;C:/Xilinx/13.3/ISE_DS/common/bin/nt64;C:/Xilinx/13.3/ISE_DS/common/lib/nt64:C:/Xilinx/13.3/ISE_DS/PlanAhead\\bin:$PATH
fi
export PATH

if [ -z "$LD_LIBRARY_PATH" ]; then
  LD_LIBRARY_PATH=
else
  LD_LIBRARY_PATH=:$LD_LIBRARY_PATH
fi
export LD_LIBRARY_PATH

HD_PWD=`dirname "$0"`
cd "$HD_PWD"

HD_LOG=runme.log
/bin/touch $HD_LOG

ISEStep="./ISEWrap.sh"
EAStep()
{
     $ISEStep $HD_LOG "$@" >> $HD_LOG 2>&1
     if [ $? -ne 0 ]
     then
         exit
     fi
}

EAStep xst -ifn "kovan.xst" -ofn "kovan.srp" -intstyle ise
