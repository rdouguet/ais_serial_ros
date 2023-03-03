# AIS DATA Messages

uint32 mmsi					# Marine Mobile Servide Identity
uint32 imo_number			# number defined by the International Maritime Organization
std_msgs/String shipname    # vessel name

uint8 nav_status            # navigation status for ClassA vessel (fishing, at anchor....)
uint8 ship_type				# ship type (fishing, towing, sailing...)

float64 longitude			# longitude in 10000 minutes
float64 latitude			# latitude in 10000 minutes

uint8 rot					# rate of turn (°)
float32 sog  				# speed over ground (°)
float32 cog					# course over ground (°)
float32 hdg					# true heading (°)
