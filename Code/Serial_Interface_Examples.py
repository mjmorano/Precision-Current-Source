from serial import Serial
from time import sleep

port = "COM7"  # Change this

CS = Serial(port, 115200, timeout=1)
sleep(1)

print(CS.readline().decode().strip())
print(CS.readline().decode().strip())

sleep(0.5)

CS.write(b"ID?\n")  # Check to make sure serial interface is working
print(CS.readline().decode().strip())

sleep(0.5)

CS.write(b"Vdd?\n")  # Get the supply voltage of the ADC. Mostly for fault testing
print(CS.readline().decode().strip())

sleep(0.5)

CS.write(b"temp?\n")  # Reads the temperature sensors. Returns [tmp117, T Resistor, T MOSFET] or [T Resistor, T MOSFET] if no tmp117 is present
print(CS.readline().decode().strip().split(", "))

sleep(0.5)

CS.write(b"curr=0.01\n") # Sets the current to 10 mA
print(CS.readline().decode().strip()) # Whenever the current is set a read needs to be done to see if the requested value was accepted. Should say "OK."

sleep(0.5)

CS.write(b"curr?\n") # Reads the current using the ADC. Value is in amps
print(CS.readline().decode().strip())

sleep(1)

CS.write(b"curr=125e-3\n") # Scientific notation can also be used for setting the current
print(CS.readline().decode().strip())

sleep(0.5)

CS.write(b"curr?\n")
print(CS.readline().decode().strip())

sleep(0.1)

CS.write(b"nAvg=10\n") # Sets the number of ADC readings averaged together for readback value. Can be up to 255, but the ADC sample rate is 16.67 Hz so setting a high value takes a long time for a reading
print(CS.readline().decode().strip()) # Make sure the value was accepted

sleep(0.1)

CS.write(b"curr?\n")
print(CS.readline().decode().strip())