# Humidity-Sniffer
This small arduino proMini project is designed for monitoring the environment humidity. If a defined threshold is exceeded a sms is send via GPRS module to a defineable phone number. Before sending the next sms hujidity must be less than the threshold. The intervall between the measurements can be defined, in the actual project it is 6 h. Very low power consumption, because GPRS module is only switched on via a MosFET if a sms must be send. So with Arduino ProMini without voltage regulator the current consumption is only about 6.5 µA during sleeping. So the project can be realized with a battery or small powerbank (abt. 2000 mAh) for a long live cycle of abt. 0.5...1 year (6h measurement cycle), depending on the number if sms calls and the measurement frequency.
