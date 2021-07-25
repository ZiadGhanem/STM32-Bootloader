import os
import glob
import serial
from elftools.common.py3compat import bytes2str
from elftools.elf.elffile import ELFFile
from functools import reduce
from operator import ixor

bootLoaderACK = b'\x79'
bootLoaderNACK = b'\x1F'

flashSectors = {
	0 : [0x08000000, 0x08003FFF],
	1 : [0x08004000, 0x08007FFF],
	2 : [0x08008000, 0x0800BFFF],
	3 : [0x0800C000, 0x0800FFFF],
	4 : [0x08010000, 0x0801FFFF],
	5 : [0x08020000, 0x0803FFFF],
	6 : [0x08040000, 0x0805FFFF],
	11: [0x080E0000, 0x080FFFFF],
	12: [0x08100000, 0x08103FFF],
	13: [0x08104000, 0x08107FFF],
	14: [0x08108000, 0x0810BFFF],
	15: [0x0810C000, 0x0810FFFF],
	16: [0x08110000, 0x0811FFFF],
	17: [0x08120000, 0x0813FFFF],
	18: [0x08140000, 0x0815FFFF],
	23: [0x081E0000, 0x081FFFFF]
}

bootLoaderCommands = {
	"Get" : b'\x00\xFF',
	"GetVersionAndProtection" : b'\x01\xFE',
	"GetID" : b'\x02\xFD',
	"ReadMemory" : b'\x11\xEE',
	"Go" : b'\x21\xDE',
	"WriteMemory" : b'\x31\xCE',
	"EraseMemory" : b'\x43\xBC',
	"WriteProtect" : b'\x63\x9C',
	"WriteUnprotect" : b'\x73\x8C',
	"ReadoutProtect" : b'\x82\x7D',
	"ReadoutUnprotect" : b'\x92\x6D' 
}

def GetSectors(startAddress, size):
	sectors = []
	endAddress = startAddress + size
	started = True
	for sectorNumber, sectorAddress  in flashSectors.items():
		if started:
			sectors.append(sectorNumber)
		if(sectorAddress[0] <= startAddress <= sectorAddress[1]):
			sectors.append(sectorNumber)
			started = True
		if(sectorAddress[0] <= endAddress <= sectorAddress[1]):
			return sectors

def GetCommand():
	# Send 0x00 + 0xFF
	ser.write(bootLoaderCommands["Get"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Receive the number of bytes (version + commands)
	numCommands = ser.read()
	# Receive the bootloader version
	bootLoaderVersion = ser.read()
	print("BootLoader Version: 0x{}".format(bootLoaderVersion.hex()))
	print("Number of supported commands: {}".format(ord(numCommands)))
	# Receive the supported commands
	supportedCommands = ser.read(ord(numCommands))
	supportedCommands = supportedCommands.hex()
	supportedCommands = ["0x" + supportedCommands[i:i+2] for i in range(0, len(supportedCommands), 2)]
	print("Supported Commands: {}".format(supportedCommands))
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def GetVersionAndProtection():
	# Send 0x01 + 0xFE
	ser.write(bootLoaderCommands["GetVersionAndProtection"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Receive the bootloader version
	bootLoaderVersion = ser.read()
	print("BootLoader Version: 0x{}".format(bootLoaderVersion.hex()))
	# Option bytes 1 and 2
	optionBytes = ser.read(2)
	optionBytes = optionBytes.hex()
	optionBytes = ["0x" + optionBytes[i:i+2] for i in range(0, len(optionBytes), 2)]
	print("Option bytes: {}".format(optionBytes))
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def GetID():
	# Send 0x02 + 0xFD
	ser.write(bootLoaderCommands["GetID"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Receive N = number of bytes - 1
	numBytes = ser.read()
	# Receive PID
	PID = ser.read(int(numBytes.hex()) + 1)
	PID = PID.hex()
	PID = ["0x" + PID[i:i+2] for i in range(0, len(PID), 2)]
	print("PID: {}".format(PID))
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def ReadMemory(startAddress, numBytes):
	if(numBytes > 255):
		print("Number of bytes must be less than or equal 256")
		exit(0)

	print("Attempting to read {} bytes from address {}".format(numBytes + 1, hex(startAddress)))
	# Send 0x11 + 0xEE
	ser.write(bootLoaderCommands["ReadMemory"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the start address (4 bytes) with checksum
	startAddress = [((startAddress >> i) & 0xFF) for i in [24, 16, 8, 0]]
	startAddress.append(reduce(ixor, startAddress))
	startAddress = bytearray(startAddress)
	ser.write(startAddress)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the number of bytes to be read (1 byte) and a checksum (1 byte)
	numBytesPlusChecksum = bytearray([numBytes, numBytes ^ 0xFF])
	ser.write(numBytesPlusChecksum)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Receive data from BL
	data = ser.read(numBytes + 1)
	data = data.hex()
	data = ["0x" + data[i:i+2] for i in range(0, len(data), 2)]
	print("Data: {}".format(data))

	
def Go(startAddress):
	print("Attempting to go to address {}".format(startAddress))
	# Send 0x21 + 0xDE
	ser.write(bootLoaderCommands["Go"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the Start Address (4 bytes + checksum)
	startAddress = [((startAddress >> i) & 0xFF) for i in [24, 16, 8, 0]]
	startAddress.append(reduce(ixor, startAddress))
	startAddress = bytearray(startAddress)
	ser.write(startAddress)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	

def WriteMemory(startAddress, data):
	numBytes = len(data)
	if(numBytes > 256):
		print("Number of bytes must be less than or equal 256")
		exit(0)

	print("Attempting to write {} bytes to address {}".format(numBytes, hex(startAddress)))
	# Send 0x31	+ 0xCE
	ser.write(bootLoaderCommands["WriteMemory"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the start address (4 bytes) & checksum
	startAddress = [((startAddress >> i) & 0xFF) for i in [24, 16, 8, 0]]
	startAddress.append(reduce(ixor, startAddress))
	startAddress = bytearray(startAddress)
	ser.write(startAddress)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the number of bytes to written (1 byte)
	ser.write(bytearray([numBytes - 1]))
	# Send the data (N + 1 bytes) and checksum
	data = list(data)
	data.append(reduce(ixor, data))
	data = bytearray(data)
	ser.write(data)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def EraseMemory(pageCodes):
	numPages = len(pageCodes)
	if(numPages == 0):
		print("No pages to be erased")
		exit(0)
	print("Attempting to erase pages: {}".format(pageCodes))
	# Send 0x43 + 0xBC
	ser.write(bootLoaderCommands["EraseMemory"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the number of pages to be erased (1 byte)
	ser.write(bytearray([numPages]))
	# Send the page numbers and checksum
	if(numPages == 1):
		pageCodes.append(pageCodes[0] ^ 0xFF)
	else:
		pageCodes.append(reduce(ixor, pageCodes))
	pageCodes = bytearray(pageCodes)
	ser.write(pageCodes)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def WriteProtect(sectorCodes):
	numSectors = len(sectorCodes)
	if(numSectors == 0):
		print("No pages to be write protected")
		exit(0)
	print("Enabling write protection on sectors: {}".format(sectorCodes))
	# Send 0x63 + 0x9C
	ser.write(bootLoaderCommands["WriteProtect"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Send the number of sectors to be protected
	ser.write(bytearray([numSectors]))
	# Send the sector codes and checksum
	if(numSectors == 1):
		sectorCodes.append(sectorCodes[0] ^ 0xFF)
	else:
		sectorCodes.append(reduce(ixor, sectorCodes))
	ser.write(sectorCodes)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	

def WriteUnprotect():
	print("Disabling write protection")
	# Send 0x73 + 0x8C
	ser.write(bootLoaderCommands["WriteUnprotect"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def ReadoutProtect():
	print("Enabling readout protection")
	# Send 0x82 + 0x7D
	ser.write(bootLoaderCommands["ReadoutProtect"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

def ReadoutUnprotect():
	print("Disabling readout protection")
	# Send 0x92 + 0x6D
	ser.write(bootLoaderCommands["ReadoutUnprotect"])
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)
	# Wait for ACK or NACK
	if(ser.read() != bootLoaderACK):
		print("Failed")
		exit(0)

# Find available serial ports
serialPorts = glob.glob('/dev/ttyACM*')
if(len(serialPorts) == 0):
	print("Connection not available")
	exit(0)
# Connect to the serial port
ser = serial.Serial(serialPorts[0], 115200, timeout = 1, parity=serial.PARITY_EVEN)
# Find elf file
os.chdir("Debug")
elfFiles = glob.glob("*.elf")
if(len(elfFiles) == 0):
	print("No elf files found")
	exit(0)
# Display available sections
print("Attempting to flash {}".format(elfFiles[0]))
with open(elfFiles[0], 'rb') as f:
	elfFile = ELFFile(f)
	print("Sections:")
	for s in elfFile.iter_sections():
		print ('[{nr}] {name} {type} {addr} {offs} {size}'.format(
			nr = s.header['sh_name'],
			name = s.name,
			type = s.header['sh_type'],
			addr = hex(s.header['sh_addr']),
			offs = hex(s.header['sh_offset']),
			size = hex(s.header['sh_size'])))
# Flash the desired sections
for sectionName in [".isr_vector", ".text", ".rodata", ".data", ".bss"]:
	print("Flashing {}".format(sectionName))
	currentSection = elffile.get_section_by_name(sectionName)
	startAddress = currentSection.header['sh_addr']
	size = currentSection.header['sh_size']
	sectors = GetSectors(startAddress, size)
	EraseMemory(sectors)
	index = 0
	while(size >= 0xFF):
		WriteMemory(startAddress, data[index: index + 0xFF])
		index += 0xFF
		startAddress += 0xFF
		size -= 0xFF
	if(size > 0):
		WriteMemory(startAddress, data[index:])	




