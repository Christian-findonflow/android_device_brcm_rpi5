#!/system/bin/sh
# GPS initialization script - toggles location to trigger GNSS HAL setCallback()
# This is a workaround for Android framework not calling setCallback() on initial boot
log -t gnss_init "Starting GPS initialization"
sleep 5
log -t gnss_init "Disabling location"
cmd location set-location-enabled false
sleep 1
log -t gnss_init "Enabling location"
cmd location set-location-enabled true
log -t gnss_init "Enabling ADAS GNSS"
cmd location set-adas-gnss-location-enabled true
log -t gnss_init "GPS initialization complete"
