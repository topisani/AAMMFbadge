ARDUINO_SKETCHBOOK  = /home/topisani/Arduino
CXXFLAGS += -I.
#BOARD_TAG           = promicro
#BOARD_SUB       = 16MHzatmega32U4
#ALTERNATE_CORE      = sparkfun
#AVRDUDE_OPTS = -V -q -F
BOARD_TAG = leonardo
ARDUINO_PORT = /dev/ttyACM0
FORCE_MONITOR_PORT = /dev/ttyACM0
include /usr/share/arduino/Arduino.mk

# vim: ft=makefile
