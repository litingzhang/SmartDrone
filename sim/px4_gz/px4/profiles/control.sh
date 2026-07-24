#!/bin/sh

SMARTDRONE_PX4_PROFILE_NAME="control"
SMARTDRONE_PROFILE_EXPECTED="SIM_GZ_EN_GPS=1 SIM_GZ_EN_FLOW=0 SIM_GZ_EN_ODOM=0 EKF2_EV_CTRL=0 EKF2_GPS_CTRL=7 EKF2_OF_CTRL=0 EKF2_HGT_REF=1 EKF2_BARO_CTRL=1 EKF2_RNG_CTRL=0 EKF2_DRAG_CTRL=0 COM_ARM_WO_GPS=1 NAV_DLL_ACT=0"

param set-default SIM_GZ_EN_GPS 1
param set-default SIM_GZ_EN_FLOW 0
param set-default SIM_GZ_EN_ODOM 0
param set-default EKF2_EV_CTRL 0
param set-default EKF2_GPS_CTRL 7
param set-default EKF2_OF_CTRL 0
param set-default EKF2_HGT_REF 1
param set-default EKF2_BARO_CTRL 1
param set-default EKF2_RNG_CTRL 0
param set-default EKF2_DRAG_CTRL 0
param set-default COM_ARM_WO_GPS 1
param set-default NAV_DLL_ACT 0
