import os
import sys
import glob
import spidev
import RPi.GPIO as GPIO
import time

# ===== Constants =====
GPIO_SYNC_PIN = 17
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 187000  # 187 KHz
SPI_MODE = 0b01     # CPOL=0, CPHA=1

ACK_BYTE = 0x79
NACK_BYTE = 0x1F
CMD_SPECIAL_COMMAND = 0x50
SPECIAL_CMD_GET_RFUC_VERSION = 0x0001
CMD_EXT_ERASE_MEMORY = 0x44
BANK_2_MASS_ERASE = 0xFFFD
CMD_WRITE_TO_MEMORY = 0x31
CMD_READ_FROM_MEMORY = 0x11
CMD_JUMP_TO_APPLICATION = 0x21

RFuC_START_ADDRESS = 0x08100000

spi = None  # Global spi variable
rfucImages = None

# ===== Functions =====
def checkPrerequisites():
    global rfucImages
    script_dir = os.path.dirname(os.path.realpath(__file__))
    rfucImages = glob.glob(os.path.join(script_dir, "RFuC_*.bin"))
    if len(rfucImages) != 1:
        raise Exception("There must be exactly 1 RFuC image in the directory with RFuCBootloader.py. Found: " + str(len(rfucImages)))

def initializeGpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_SYNC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def initializeSpi():
    global spi
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = SPI_MODE
    spi.bits_per_word = 8

def cleanup():
    spi.close()
    GPIO.cleanup()
    print("RFuC bootloader app finished with success.")
    sys.exit(0)  # Exit script with success status

def SpiTxRxByte(byte):
    return spi.xfer2([byte])[0]  # Returns the received byte

def waitForSync():
    print("Waiting for RFuC bootloader synchronization signal...")
    count = 0
    while count < 600:
        if GPIO.input(GPIO_SYNC_PIN) == GPIO.HIGH:
            print("Synchronization signal received. Starting RFuC bootloader communication...")
            break
        time.sleep(0.1) # Sleep to avoid busy-waiting. Units are in seconds.
    if count >= 600:
        raise Exception("Sync Frame not acknowledged")

    SpiTxRxByte(0x5A)
    count = 0
    while (not (SpiTxRxByte(0x5A) == 0xA5) and (GPIO.input(GPIO_SYNC_PIN) == GPIO.HIGH)) and (count < 600):
        count += 1
        time.sleep(0.1) # Sleep to avoid busy-waiting. Units are in seconds.
    if count >= 600:
        raise Exception("Sync Frame not acknowledged")
    else:
        acknowledgeFrame()
    
def sendOnesComplement(byte):
    SpiTxRxByte(~byte & 0xFF)

def acknowledgeFrame():
    SpiTxRxByte(0x00)
    if SpiTxRxByte(0x00) != ACK_BYTE:
        raise Exception("ACK not received")
    SpiTxRxByte(ACK_BYTE)

def getBootloaderVersion():
    # Request Version
    SpiTxRxByte(0x5A)
    SpiTxRxByte(0x01)
    sendOnesComplement(0x01)
    acknowledgeFrame()

    # Get Version
    SpiTxRxByte(0x00)
    version = SpiTxRxByte(0x00)
    acknowledgeFrame()

    # Print the bootloader version
    major_version = version >> 4
    minor_version = version & 0x0F
    print(f"Bootloader Version: {major_version}.{minor_version}")

    # Check that version is at least 1.0
    if major_version < 1:
        raise Exception("Bootloader version must be at least 1.0")

def getDeviceID():
    # Request ID
    SpiTxRxByte(0x5A)
    SpiTxRxByte(0x02)
    sendOnesComplement(0x02)
    acknowledgeFrame()

    # Get ID
    SpiTxRxByte(0x00)
    preceding_byte = SpiTxRxByte(0x00)
    device_id = (SpiTxRxByte(0x00) << 8) | SpiTxRxByte(0x00)
    acknowledgeFrame()

    # Print the device ID
    print(f"Device ID: 0x{device_id:03X}")

    # Verify the preceding byte and device ID
    if preceding_byte != 0x01 or device_id != 0x470:
        raise Exception(f"Unexpected Device ID or preceding byte: received 0x{device_id:03X}, expected 0x470")

    return device_id

def getRfucVersion():
    # Extract version info from the filename
    expectedFilename = os.path.basename(rfucImages[0])
    expectedVersionInfo = expectedFilename[:-4].split('_')[1:]

    # Send Special Command header
    SpiTxRxByte(0x5A)
    SpiTxRxByte(CMD_SPECIAL_COMMAND)
    sendOnesComplement(CMD_SPECIAL_COMMAND)
    acknowledgeFrame()

    # Send Get RFuC Version special command (2 bytes)
    high_byte = (SPECIAL_CMD_GET_RFUC_VERSION >> 8) & 0xFF
    low_byte = SPECIAL_CMD_GET_RFUC_VERSION & 0xFF
    SpiTxRxByte(high_byte)  # Send the high byte
    SpiTxRxByte(low_byte)   # Send the low byte
    SpiTxRxByte(high_byte ^ low_byte) # Compute and send the XOR checksum of the command bytes
    acknowledgeFrame()

    # Send number of bytes of data to send
    high_byte = 0x00
    low_byte = 0x00
    SpiTxRxByte(high_byte)  # Send the high byte
    SpiTxRxByte(low_byte)   # Send the low byte
    SpiTxRxByte(high_byte ^ low_byte) # Compute and send the XOR checksum of the command bytes
    acknowledgeFrame()

    # Get number of data bytes to receive
    dataSize = (SpiTxRxByte(0x00) << 8) | SpiTxRxByte(0x00)
    # Validate dataSize == 100?? bytes of data
    #print("dataSize = " + str(dataSize))

    # Retrieve the RFuC Version data
    versionInfo = bytearray()
    for _ in range(dataSize):
        versionInfo.append(SpiTxRxByte(0x00))

    # Get number of status bytes to receive
    StatusSize = (SpiTxRxByte(0x00) << 8) | SpiTxRxByte(0x00)
    # Validate StatusSize == 0 bytes of status
    #print("StatusSize = " + str(StatusSize))
    acknowledgeFrame()

    # Print the received data
    #print(f"Received RFuC Version Info: {versionInfo}")
    
    # Check if the version information is of valid format
    try:
        version_parts = versionInfo.decode().split('\x00')
    except UnicodeDecodeError:
        print("No RFuC firmware detected. Updating RFuC with latest image...")
        return False

    # Parse version_parts
    sha1Hash = version_parts[0]
    validity = version_parts[-2]
    RfucDate, compileDate = "", ""
    date_counter = 0
    for part in version_parts[1:-2]:
        if '-' in part or ':' in part:  # Check if it's a date
            if date_counter == 0:  # If first date, set RfucDate
                RfucDate = part.replace(':', '-').replace(' ', '-')
                date_counter += 1
            elif date_counter == 1:  # If second date, set compileDate
                compileDate = part.replace(':', '-').replace(' ', '-')
                date_counter += 1
    if RfucDate.endswith('-MST'):
        RfucDate = RfucDate[:-4]
    if compileDate.endswith('-'):
        compileDate = compileDate[:-1]

    # Print the received data
    print(f"Received RFuC Version Info:")
    print(f"  sha1Hash: {sha1Hash}")
    print(f"  validity: {validity}")
    print(f"  RfucDate: {RfucDate}")
    print(f"  compileDate: {compileDate}")

    receivedVersionInfo = [sha1Hash, validity, compileDate]
    print(f"Received RFuC Version Info does not match the expected version.\n  Expected: {expectedVersionInfo}\n  Received: {receivedVersionInfo}")
    if receivedVersionInfo != expectedVersionInfo:
        print("Updating RFuC with latest image...")
        return False

    print("The RFuC is already running the latest firmware.")
    return True

def massEraseRfucApplication():
    # Send Memory Erase header
    SpiTxRxByte(0x5A)
    SpiTxRxByte(CMD_EXT_ERASE_MEMORY)
    sendOnesComplement(CMD_EXT_ERASE_MEMORY)
    acknowledgeFrame()

    # Send two bytes representing the mass erase of Bank 2
    SpiTxRxByte((BANK_2_MASS_ERASE >> 8) & 0xFF) # MSB
    SpiTxRxByte(BANK_2_MASS_ERASE & 0xFF) # LSB
    #acknowledgeFrame()

    # Compute checksum for special erase
    checksum = ((BANK_2_MASS_ERASE >> 8) & 0xFF) ^ (BANK_2_MASS_ERASE & 0xFF)
    SpiTxRxByte(checksum)

    # Wait to allow time for mass erase operation.
    counter = 0
    timeout_limit = 5
    while True:
        if GPIO.input(GPIO_SYNC_PIN) == GPIO.HIGH:
            break
        time.sleep(0.1) # Units are in seconds.
        counter += 1
        if counter >= timeout_limit:
            raise TimeoutError("Mass Erase of RFuC Timed Out after 500ms.")

    acknowledgeFrame()

    print("Mass Erase of RFuC (Bank 2) Complete.")

def sendAddress(address):
    Byte3 = (address >> 24) & 0xFF
    Byte2 = (address >> 16) & 0xFF
    Byte1 = (address >> 8) & 0xFF
    Byte0 = address & 0xFF
    SpiTxRxByte(Byte3)
    SpiTxRxByte(Byte2)
    SpiTxRxByte(Byte1)
    SpiTxRxByte(Byte0)

    # Compute checksum
    checksum = (Byte3 ^ Byte2 ^ Byte1 ^ Byte0)
    SpiTxRxByte(checksum)
    acknowledgeFrame()

def updateToLatestRfucApplication():
    CHUNK_SIZE = 256
    BIN_FILE_PATH = rfucImages[0]

    with open(BIN_FILE_PATH, "rb") as bin_file:
        address = RFuC_START_ADDRESS
        chunk = bin_file.read(CHUNK_SIZE)
        while chunk:
            # Skip write operation if the chunk contains only blank values
            if all(byte == 0x00 for byte in chunk):
                address += CHUNK_SIZE # Update the address for the next chunk
                chunk = bin_file.read(CHUNK_SIZE) # Read the next chunk
                continue

            # Send Write Memory header
            SpiTxRxByte(0x5A)
            SpiTxRxByte(CMD_WRITE_TO_MEMORY)
            sendOnesComplement(CMD_WRITE_TO_MEMORY)
            acknowledgeFrame()

            # Send data frame start address
            sendAddress(address)

            # Send data frame
            ## Send number of bytes to be written
            NumberOfBytesToWrite = len(chunk) - 1
            SpiTxRxByte(NumberOfBytesToWrite)
            ## Send data to be written and calculate checksum
            checksum = NumberOfBytesToWrite
            for byte in chunk:
                SpiTxRxByte(byte)
                checksum = checksum ^ byte
            ## Send checksum
            SpiTxRxByte(checksum)

            # Wait to allow time for chunk write operation.
            counter = 0
            timeout_limit = 10
            while True:
                if GPIO.input(GPIO_SYNC_PIN) == GPIO.HIGH:
                    break
                time.sleep(0.01) # Units are in seconds.
                counter += 1
                if counter >= timeout_limit:
                    raise TimeoutError("Mass Erase of RFuC Timed Out after 100 ms.")

            acknowledgeFrame()

            address += CHUNK_SIZE # Update the address for the next chunk
            chunk = bin_file.read(CHUNK_SIZE) # Read the next chunk

    print("RFuC Firmware Update Complete.")

def verifyFlashedRfuc():
    CHUNK_SIZE = 256
    BIN_FILE_PATH = rfucImages[0]

    with open(BIN_FILE_PATH, "rb") as bin_file:
        address = RFuC_START_ADDRESS
        chunk = bin_file.read(CHUNK_SIZE)
        while chunk:
            # Send Write Memory header
            SpiTxRxByte(0x5A)
            SpiTxRxByte(CMD_READ_FROM_MEMORY)
            sendOnesComplement(CMD_READ_FROM_MEMORY)
            acknowledgeFrame()

            sendAddress(address) # Send data frame start address
            SpiTxRxByte(len(chunk)) # Send number of bytes to be read
            sendOnesComplement(len(chunk)) # Send checksum
            acknowledgeFrame()

            # Receive requested data
            ReadData = bytearray()
            for _ in range(len(chunk)):
                ReadData.append(SpiTxRxByte(0x00))

            # Validate data
            LatestData = bytearray()
            for byte in chunk:
                LatestData.append(byte)
            if ReadData != LatestData:
                print("address: 0x" + str(address))
                print("ReadData: " + str(ReadData))
                print("LatestData: " + str(LatestData))
                raise Exception("ERROR: Failed to validate RFuC firmware update was successful")

            address += CHUNK_SIZE # Update the address for the next chunk
            chunk = bin_file.read(CHUNK_SIZE) # Read the next chunk

    print("RFuC firmware update was successful")

def jumpToRfucApplication():
    # Send Write Memory header
    SpiTxRxByte(0x5A)
    SpiTxRxByte(CMD_JUMP_TO_APPLICATION)
    sendOnesComplement(CMD_JUMP_TO_APPLICATION)
    acknowledgeFrame()

    # Send data frame start address
    sendAddress(RFuC_START_ADDRESS)
    print("Jumping to RFuC Application...")

def main():
    global spi, rfucImages

    checkPrerequisites()
    initializeGpio()
    initializeSpi()
    waitForSync()
    getBootloaderVersion()
    getDeviceID()
    if getRfucVersion():
        jumpToRfucApplication()
        cleanup()
    massEraseRfucApplication()
    updateToLatestRfucApplication()
    #verifyFlashedRfuc()
    jumpToRfucApplication()
    cleanup()

if __name__ == "__main__":
    main()


