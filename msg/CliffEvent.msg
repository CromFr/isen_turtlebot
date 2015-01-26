# Provides a cliff event.
# This message is generated whenever a particular cliff is floor or cliff.
# Note that, despite bumper field on SensorState messages, state field is not a
# bitmask, but the new state of a single sensor.

# sensor
uint8 LEFT   = 0
uint8 CENTER = 1
uint8 RIGHT  = 2

# state
uint8 FLOOR = 0
uint8 CLIFF  = 1

# bottom


uint8 sensor
uint8 state
uint8 bottom
