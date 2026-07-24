#!/bin/sh
set -e

: "${SMARTDRONE_MAVLINK_REMOTE_PORT:?missing SmartDrone MAVLink remote port}"
: "${SMARTDRONE_MAVLINK_LOCAL_PORT:?missing PX4 MAVLink local port}"
: "${SMARTDRONE_MAVLINK_HOST:?missing SmartDrone MAVLink host}"

# The generated rcS sources px4-alias.sh and applies the selected airframe profile
# before estimator startup.
# shellcheck disable=SC1090,SC2154
. "${R}etc/init.d-posix/rcS"
# rcS defines a PX4 compatibility alias named "set"; bypass it here so shell
# error handling is restored for the validation and MAVLink setup below.
command set -e

VerifyProfileParameters()
{
    validationFailed=0
    for profileSetting in ${SMARTDRONE_PROFILE_EXPECTED:-}; do
        profileParam=${profileSetting%%=*}
        expectedValue=${profileSetting#*=}
        param show "$profileParam" || true
        if ! param compare "$profileParam" "$expectedValue"; then
            echo "ERROR [smartdrone] $profileParam does not equal $expectedValue"
            validationFailed=1
        fi
    done
    [ "$validationFailed" -eq 0 ]
}

echo "INFO  [smartdrone] PX4 parameter profile: ${SMARTDRONE_PX4_PROFILE_NAME:-unknown}"
if ! VerifyProfileParameters; then
    echo "ERROR [smartdrone] refusing to start MAVLink with an invalid parameter profile"
    exit 1
fi

# Stop the stock onboard and GCS instances. The latter sends MAVLink datagrams
# to UDP 14550, which is reserved for SmartDrone's TLV command server here.
# Free the requested local and remote ports as well: either may overlap another
# stock SITL link when non-default ports are selected.
mavlink stop -u 14580 || true
mavlink stop -u 18570 || true
mavlink stop -u "$SMARTDRONE_MAVLINK_LOCAL_PORT" || true
mavlink stop -u "$SMARTDRONE_MAVLINK_REMOTE_PORT" || true
mavlink start -x -u "$SMARTDRONE_MAVLINK_LOCAL_PORT" -r 4000000 -f \
    -m onboard -o "$SMARTDRONE_MAVLINK_REMOTE_PORT" \
    -t "$SMARTDRONE_MAVLINK_HOST"
mavlink stream -r 30 -s LOCAL_POSITION_NED -u "$SMARTDRONE_MAVLINK_LOCAL_PORT"
mavlink stream -r 30 -s ATTITUDE -u "$SMARTDRONE_MAVLINK_LOCAL_PORT"
mavlink stream -r 10 -s ESTIMATOR_STATUS -u "$SMARTDRONE_MAVLINK_LOCAL_PORT"
mavlink stream -r 5 -s EXTENDED_SYS_STATE -u "$SMARTDRONE_MAVLINK_LOCAL_PORT"
mavlink stream -r 5 -s SYS_STATUS -u "$SMARTDRONE_MAVLINK_LOCAL_PORT"

mavlink status
