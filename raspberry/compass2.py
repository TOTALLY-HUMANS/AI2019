import smbus

bus = smbus.SMBus(1)

DEVICE_ADDRESS = 0x1e

data = []
for i in range(0, 100):
    data.append(bus.read_byte_data(DEVICE_ADDRESS, i))

print(data)

DEVICE_ADDRESS = 0x1d

data = []
for i in range(0, 100):
    data.append(bus.read_byte_data(DEVICE_ADDRESS, i))

print(data)
