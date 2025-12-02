import hid

for device in hid.enumerate():
    print(f"Device: {device['product_string']}")
    print(f"Vendor ID: {hex(device['vendor_id'])}")
    print(f"Product ID: {hex(device['product_id'])}")
    print("---")

#Vendor ID:
VID = 0x3285
#Product ID:
PID = 0xc03

while True:
    device = hid.device()
    device.open(VID, PID)
    data = device.read(64)
    print(data)
