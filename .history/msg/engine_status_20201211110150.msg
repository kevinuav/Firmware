uint64 timestamp	# time since system start (microseconds)

float32 temp
float32 vol
uint64 rpm		# unique device ID for the sensor that does not change between power cycles
uint8 status		# ADC channel IDs, negative for non-existent, TODO: should be kept same as array index
uint8 fault		# ADC channel raw value, accept negative value, valid if channel ID is positive
uint16 running_time		# ADC channel resolution
float32 pump
float64 fuel_consumed	# ADC channel voltage reference, use to calculate LSB voltage(lsb=scale/resolution)
float64 engine_load
bool engine_failure
