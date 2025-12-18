import serial
import time

print(serial.__file__)

# Instruction Definitions
# Each definition corresponds to an instruction within the DYNx drive. These are universal across all drives
# that support RS485/232 communication

goAbsolutePosition = 0x01
goRelativePosition = 0x03
goConstantSpeed = 0x0a
goSquareWave = 0x0b
goSineWave = 0x0c

generalRead = 0x0e

setEncoderOrigin = 0x00
setDriveID = 0x05
setMainGain = 0x10
setSpeedGain = 0x11
setIntegrationGain = 0x12
setTorqueConstant = 0x13
setHighSpeed = 0x14
setHighAccel = 0x15
setPosOnRange = 0x16
setGearNum = 0x17
setWaveFreq = 0x0d

# Config Modes
# lookup table for configuration modes
CONFIG_MODES = {
    0b00000000: "RS232 Mode, Relative Position, Position Servo, Drive Servo",
    0b00000001: "CW/CCW Mode, Relative Position, Position Servo, Drive Servo",
    0b00000010: "Pulse/Dir Mode, Relative Position, Position Servo, Drive Servo",
    0b00000011: "Analog Mode, Relative Position, Position Servo, Drive Servo",
    0b00000100: "RS232 Mode, Absolute Position, Position Servo, Drive Servo",
    0b00000101: "CW/CCW Mode, Absolute Position, Position Servo, Drive Servo",
    0b00000110: "Pulse/Dir Mode, Absolute Position, Position Servo, Drive Servo",
    0b00000111: "Analog Mode, Absolute Position, Position Servo, Drive Servo",
    0b00001000: "RS232 Mode, Relative Position, Speed Servo, Drive Servo",
    0b00001001: "CW/CCW Mode, Relative Position, Speed Servo, Drive Servo",
    0b00001010: "Pulse/Dir Mode, Relative Position, Speed Servo, Drive Servo",
    0b00001011: "Analog Mode, Relative Position, Speed Servo, Drive Servo",
    0b00001100: "RS232 Mode, Absolute Position, Speed Servo, Drive Servo",
    0b00001101: "CW/CCW Mode, Absolute Position, Speed Servo, Drive Servo",
    0b00001110: "Pulse/Dir Mode, Absolute Position, Speed Servo, Drive Servo",
    0b00001111: "Analog Mode, Absolute Position, Speed Servo, Drive Servo",
    0b00010000: "RS232 Mode, Relative Position, Torque Servo, Drive Servo",
    0b00010001: "CW/CCW Mode, Relative Position, Torque Servo, Drive Servo",
    0b00010010: "Pulse/Dir Mode, Relative Position, Torque Servo, Drive Servo",
    0b00010011: "Analog Mode, Relative Position, Torque Servo, Drive Servo",
    0b00010100: "RS232 Mode, Absolute Position, Torque Servo, Drive Servo",
    0b00010101: "CW/CCW Mode, Absolute Position, Torque Servo, Drive Servo",
    0b00010110: "Pulse/Dir Mode, Absolute Position, Torque Servo, Drive Servo",
    0b00010111: "Analog Mode, Absolute Position, Torque Servo, Drive Servo",
    0b00100000: "RS232 Mode, Relative Position, Position Servo, Free Drive",
    0b00100001: "CW/CCW Mode, Relative Position, Position Servo, Free Drive",
    0b00100010: "Pulse/Dir Mode, Relative Position, Position Servo, Free Drive",
    0b00100011: "Analog Mode, Relative Position, Position Servo, Free Drive",
    0b00100100: "RS232 Mode, Absolute Position, Position Servo, Free Drive",
    0b00100101: "CW/CCW Mode, Absolute Position, Position Servo, Free Drive",
    0b00100110: "Pulse/Dir Mode, Absolute Position, Position Servo, Free Drive",
    0b00100111: "Analog Mode, Absolute Position, Position Servo, Free Drive",
    0b00101000: "RS232 Mode, Relative Position, Speed Servo, Free Drive",
    0b00101001: "CW/CCW Mode, Relative Position, Speed Servo, Free Drive",
    0b00101010: "Pulse/Dir Mode, Relative Position, Speed Servo, Free Drive",
    0b00101011: "Analog Mode, Relative Position, Speed Servo, Free Drive",
    0b00101100: "RS232 Mode, Absolute Position, Speed Servo, Free Drive",
    0b00101101: "CW/CCW Mode, Absolute Position, Speed Servo, Free Drive",
    0b00101110: "Pulse/Dir Mode, Absolute Position, Speed Servo, Free Drive",
    0b00101111: "Analog Mode, Absolute Position, Speed Servo, Free Drive",
    0b00110000: "RS232 Mode, Relative Position, Torque Servo, Free Drive",
    0b00110001: "CW/CCW Mode, Relative Position, Torque Servo, Free Drive",
    0b00110010: "Pulse/Dir Mode, Relative Position, Torque Servo, Free Drive",
    0b00110011: "Analog Mode, Relative Position, Torque Servo, Free Drive",
    0b00110100: "RS232 Mode, Absolute Position, Torque Servo, Free Drive",
    0b00110101: "CW/CCW Mode, Absolute Position, Torque Servo, Free Drive",
    0b00110110: "Pulse/Dir Mode, Absolute Position, Torque Servo, Free Drive",
    0b00110111: "Analog Mode, Absolute Position, Torque Servo, Free Drive",
}

# Create a numerical reference table
CONFIG_REFERENCE = {i + 1: (byte, desc) for i, (byte, desc) in enumerate(CONFIG_MODES.items())}

# Define lookup tables
statusFlags = {
    0b00000001: "Motor Busy Or Position Error",
    0b00000010: "Motor Free",
    0b00100000: "Motion Running",
    0b01000000: "Pin 2 Status For Cnc Zero Detection"
}

alarmCodes = {
    0b000: "No Alarm",
    0b001: "Motor Lost Phase Alarm",
    0b010: "Motor Over Current Alarm",
    0b011: "Motor Overheat Or Over Power Alarm",
    0b100: "CRC Error Command Rejected"
}

# Read/Set codes
getDriveID = 0x06
getDrvieConfig = 0x08
getDriveStatus = 0x09
getMainGain = 0x18
getDerivGain = 0x19
getIntGain = 0x1a
getTrqConst = 0x1b
getMaxSpeed = 0x1c
getMaxAccel = 0x1d
getPosOnRange = 0x1e
getGearNum = 0x1f

isMainGain = 0x10
isSpeedGain = 0x11
isIntegrationGain = 0x12
isTorqueConstant = 0x13
isHighSpeed = 0x14
isHighAccel = 0x15
isDriveID = 0x16
isPosOnRange = 0x17
isStatus = 0x19
isGearNumber = 0x18
isConfig = 0x1a
isAbsolutePosition = 0x1b
isMotorSpeed = 0x1d
isTorqueCurrent = 0x1e
isDriveReset = 0x1c
isDriveEnable = 0x20
isDriveDisable = 0x21


class DmmDriver:
    """
    Wrapper class for DMM_Lib_V1 functionality.
    Encapsulates serial port, buffers, state flags, and provides all commands.
    """

    def __init__(self, port, baudrate=38400, timeout=0.3):
        try:
            self.serialConnection = serial.Serial(port, baudrate, timeout=timeout, write_timeout=0.5)
        except (serial.SerialException, OSError) as e:
            self.serialConnection = None
        # Buffers and pointers
        self.inputBuffer = []
        self.readPackage = []
        self.inBufferTopPtr = 0
        self.inBufferBtmPtr = 0
        self.readNum = 0
        self.readPackageLength = 0
        self.outputBuffer = []
        self.outBufferTopPtr = 0
        self.outBufferBtmPtr = 0
        # Global function tracker
        self.globalFunc = 0
        # Flags and data fields
        self.MotorPosition32ReadyFlag = 0x00
        self.MotorSpeed32ReadyFlag = 0x00
        self.MotorTorqueCurrentReadyFlag = 0x00
        self.DriveStatusFlag = 0x00
        self.motorPos32 = 0
        self.motorSpeed32 = 0
        self.motorTorqueCurrent = 0
        self.driverStatus = 0
        self.driverMainGain = 0
        self.driverIntGain = 0
        self.driverSpeedGain = 0
        self.driverTorqueConstant = 0
        self.driverMaxSpeed = 0
        self.driverMaxAccel = 0
        self.driverGearNumber = 0
        self.driverConfigByte = 0
        self.driverIDNumber = 0
        self.driverOnRange = 0
       
    '''
    Date: 10 October 2024
    Author: Aidan Drescher
    Function: turnConstSpeed
    Parameters:
        ID -> Drive ID of the DYN Servo drive being addressed
        spd -> The speed to spin the shaft at in RPM
    Description: Sets the global function code and sends a package to turn at a constant speed.
    '''
    def turnConstSpeed(self, ID, spd):
        self.globalFunc = goConstantSpeed
        self.sendPackage(ID, spd)

    '''
    Date: 10 October 2024
    Author: Aidan Drescher
    Function: moveAbs32
    Parameters:
        ID -> Drive ID
        pos -> Encoder position value to move to.
    Description: Sets global function and sends package to move to absolute position.
    '''
    def moveAbs32(self, ID, pos):
        self.globalFunc = goAbsolutePosition
        self.sendPackage(ID, pos)

    '''
    Date: 10 October 2024
    Author: Aidan Drescher
    Function: moveRel32
    Parameters:
        ID -> Drive ID
        pos -> Encoder pulses to move by.
    Description: Sets global function and sends package to move relatively.
    '''
    def moveRel32(self, ID, pos):
        self.globalFunc = goRelativePosition
        self.sendPackage(ID, pos)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: moveSquareWave
    Parameters:
        ID -> Drive ID
        amplitude -> Amplitude of square wave in encoder pulses
    Description: Sets up square wave motion for the drive.
    '''
    def moveSquareWave(self, ID, amplitude):
        self.globalFunc = goSquareWave
        self.sendPackage(ID, amplitude)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: moveSineWave
    Parameters:
        ID -> Drive ID
        amplitude -> Amplitude of sine wave in encoder pulses
    Description: Sets up sine wave motion for the drive.
    '''
    def moveSineWave(self, ID, amplitude):
        self.globalFunc = goSineWave
        self.sendPackage(ID, amplitude)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setWaveFrequency
    Parameters:
        ID -> Drive ID
        frequency -> Frequency in Hz
    Description: Sets frequency for square/sine wave motion.
    '''
    def setWaveFrequency(self, ID, frequency):
        self.globalFunc = setWaveFreq
        self.sendPackage(ID, frequency)

    ''' 
    Date: 19 February 2025
    Author: Aidan Drescher
    Function: setDriveIDNum
    Parameters:
        ID -> Current drive ID
        newID -> New drive ID
    Description: Sets the ID for the drive.
    '''
    def setDriveIDNum(self, ID, newID):
        self.globalFunc = setDriveID
        self.sendPackage(ID, newID)
        
    ''' 
    Date: 5 May 2025
    Author: Aidan Drescher
    Function: setPosOnRange
    Parameters:
        ID -> Current drive ID
        range -> New value for PosOnRange
    Description: Sets the ID for the drive.
    '''
    def setPosOnRange(self, ID, range):
        self.globalFunc = setPosOnRange
        self.sendPackage(ID, range)

    ''' 
    Date: 10 October 2024
    Author: Aidan Drescher
    Function: readMotorPosition32
    Parameters:
        ID -> Drive ID
    Description: Reads motor position until ready flag cleared.
    '''
    def readMotorPosition32(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isAbsolutePosition)
        self.MotorPosition32ReadyFlag = 0xff
        while self.MotorPosition32ReadyFlag != 0x00:
            self.readInputPackage()

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: readMotorSpeed32
    Parameters:
        ID -> Drive ID
    Description: Reads motor speed until ready flag cleared.
    '''
    def readMotorSpeed32(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isMotorSpeed)
        self.MotorSpeed32ReadyFlag = 0xff
        while self.MotorSpeed32ReadyFlag != 0x00:
            self.readInputPackage()

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: readMotorTorqueCurrent
    Parameters:
        ID -> Drive ID
    Description: Reads motor current until ready flag cleared.
    '''
    def readMotorTorqueCurrent(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isTorqueCurrent)
        self.MotorTorqueCurrentReadyFlag = 0xff
        while self.MotorTorqueCurrentReadyFlag != 0x00:
            self.readInputPackage()

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: readDriveStatus
    Parameters:
        ID -> Drive ID
    Description: Reads drive status once.
    '''
    def readDriveStatus(self, ID):
        self.globalFunc = getDriveStatus
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isStatus
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readMainGain
    Parameters:
        ID -> Drive ID
    Description: Reads main gain once.
    '''
    def readMainGain(self, ID):
        self.globalFunc = getMainGain
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isMainGain
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readIntGain
    Parameters:
        ID -> Drive ID
    Description: Reads integration gain once.
    '''
    def readIntGain(self, ID):
        self.globalFunc = getIntGain
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isIntegrationGain
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readDerivGain
    Parameters:
        ID -> Drive ID
    Description: Reads derivative gain 
    '''
    def readDerivGain(self, ID):
        self.globalFunc = getDerivGain
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isSpeedGain
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readTorqueConst
    Parameters:
        ID -> Drive ID
    Description: Reads torque constant once.
    '''
    def readTorqueConst(self, ID):
        self.globalFunc = getTrqConst
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isTorqueConstant
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readMaxSpeed
    Parameters:
        ID -> Drive ID
    Description: Reads max speed once.
    '''
    def readMaxSpeed(self, ID):
        self.globalFunc = getMaxSpeed
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isHighSpeed
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readMaxAccel
    Parameters:
        ID -> Drive ID
    Description: Reads max acceleration once.
    '''
    def readMaxAccel(self, ID):
        self.globalFunc = getMaxAccel
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isHighAccel
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readGearNumber
    Parameters:
        ID -> Drive ID
    Description: Reads gear number once.
    '''
    def readPosOnRange(self, ID):
        self.globalFunc = getPosOnRange
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isPosOnRange
        self.readInputPackage()

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: readGearNumber
    Parameters:
        ID -> Drive ID
    Description: Reads gear number once.
    '''
    def readGearNumber(self, ID):
        self.globalFunc = getGearNum
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isGearNumber
        self.readInputPackage()

    ''' 
    Date: 19 February 2025
    Author: Aidan Drescher
    Function: readDriveConfig
    Parameters:
        ID -> Drive ID
    Description: Reads configuration byte once.
    '''
    def readDriveConfig(self, ID):
        self.globalFunc = getDrvieConfig
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isConfig
        self.readInputPackage()

    ''' 
    Date: 19 February 2025
    Author: Aidan Drescher
    Function: readDriveID
    Parameters:
        ID -> Drive ID
    Description: Reads drive ID once.
    '''
    def readDriveID(self, ID):
        self.globalFunc = getDriveID
        self.sendPackage(ID, 0x00)
        time.sleep(0.025)
        self.globalFunc = isDriveID
        self.readInputPackage()

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: driveReset
    Parameters:
        ID -> Drive ID
    Description: Sends reset command to drive.
    '''
    def driveReset(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isDriveReset)

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: driveEnable
    Parameters:
        ID -> Drive ID
    Description: Sends enable command to drive.
    '''
    def driveEnable(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isDriveEnable)

    ''' 
    Date: 12 November 2024
    Author: Aidan Drescher
    Function: driveDisable
    Parameters:
        ID -> Drive ID
    Description: Sends disable command to drive.
    '''
    def driveDisable(self, ID):
        self.globalFunc = generalRead
        self.sendPackage(ID, isDriveDisable)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setMaxSpeed
    Parameters:
        ID -> Drive ID
        speed -> 1-127 range
    Description: Sets the maximum speed for the motor addressed.
    '''
    def setMaxSpeed(self, ID, speed):
        self.globalFunc = setHighSpeed
        self.sendPackage(ID, speed)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setMaxAccel
    Parameters:
        ID -> Drive ID
        accel -> 1-127 range
    Description: Sets the maximum acceleration for the motor addressed.
    '''
    def setMaxAccel(self, ID, accel):
        self.globalFunc = setHighAccel
        self.sendPackage(ID, accel)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setMainGain
    Parameters:
        ID -> Drive ID
        value -> 1-127 range
    Description: Sets the proportional (Main) gain for the motor addressed.
    '''
    def setMainGain(self, ID, value):
        self.globalFunc = setMainGain
        self.sendPackage(ID, value)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setIntGain
    Parameters:
        ID -> Drive ID
        value -> 1-127 range
    Description: Sets the integration gain for the motor addressed.
    '''
    def setIntGain(self, ID, value):
        self.globalFunc = setIntegrationGain
        self.sendPackage(ID, value)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setSpeedGain
    Parameters:
        ID -> Drive ID
        value -> 1-127 range
    Description: Sets the derivative (Speed) gain for the motor addressed.
    '''
    def setSpeedGain(self, ID, value):
        self.globalFunc = setSpeedGain
        self.sendPackage(ID, value)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setTorqueFiltConst
    Parameters:
        ID -> Drive ID
        value -> 1-127 range
    Description: Sets the torque filter constant for the motor addressed.
    '''
    def setTorqueFiltConst(self, ID, value):
        self.globalFunc = setTorqueConstant
        self.sendPackage(ID, value)

    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setGearNumber
    Parameters:
        ID -> Drive ID
        value -> 1-127 range
    Description: Sets the gear number constant for the motor addressed.
    '''
    def setGearNumber(self, ID, value):
        self.globalFunc = setGearNum
        self.sendPackage(ID, value)
    
    ''' 
    Date: 18 February 2025
    Author: Aidan Drescher
    Function: setEncoderZero
    Parameters:
        ID -> Drive ID
    Description: Sets the encoder position of the motor addressed to zero.
    '''
    def setEncoderZero(self, ID):
        self.globalFunc = setEncoderOrigin
        self.sendPackage(ID, 0)  # Works only for the DYN5 and multiturn encoders

    def sendPackage(self, ID, displacement):
        """
        Constructs and sends a data package by setting and adjusting the bytes in the packet
        based on the provided ID and displacement values. Ensures optimized packet length.
        """
        packet = [0x80] * 8
        packet[0] = ID & 0x7F
        functionCode = self.globalFunc & 0x1F
        tempDisplacement = displacement & 0x0FFFFFFF
        for i in range(5, 1, -1):
            packet[i] += tempDisplacement & 0x7F
            tempDisplacement >>= 7
        packetLength = 7
        if displacement >> 20 in {0x00000000, 0xFFFFFFFF}:
            packet[2:5] = packet[3:6]
            packetLength = 6
        if displacement >> 13 in {0x00000000, 0xFFFFFFFF}:
            packet[2:4] = packet[3:5]
            packetLength = 5
        if displacement >> 6 in {0x00000000, 0xFFFFFFFF}:
            packet[2:3] = packet[3:4]
            packetLength = 4
        packet[1] += (packetLength - 4) * 32 + functionCode
        self.makeCRCsend(packetLength, packet)

    def makeCRCsend(self, packetLength, packet):
        """
        Calculates an error-check byte, appends it, and writes all packet bytes over serial.
        """
        errorCheck = 0
        for i in range(packetLength - 1):
            self.outputBuffer.append(packet[i])
            self.outBufferTopPtr += 1
            errorCheck += packet[i]
        errorCheck = (errorCheck | 0x80) & 0xFF
        self.outputBuffer.append(errorCheck)
        self.outBufferTopPtr += 1
        while self.outBufferBtmPtr != self.outBufferTopPtr:
            byte = self.outputBuffer[self.outBufferBtmPtr]
            self.serialConnection.write(bytes([byte]))
            self.outBufferBtmPtr += 1

    def readInputPackage(self):
        """
        Reads incoming data from UART, assembles packets, and dispatches to getFunction.
        """
        while self.serialConnection.in_waiting:
            b = self.serialConnection.read(1)[0]
            self.inputBuffer.append(b)
            self.inBufferTopPtr += 1
        while self.inBufferBtmPtr != self.inBufferTopPtr:
            b = self.inputBuffer[self.inBufferBtmPtr]
            self.inBufferBtmPtr += 1
            isNew = b & 0x80
            if isNew == 0:
                self.readNum = 0
                self.readPackageLength = 0
                self.readPackage.clear()
            if isNew == 0 or self.readNum > 0:
                self.readPackage.append(b)
                self.readNum += 1
                if self.readNum == 2:
                    lenInd = (b >> 5) & 0x03
                    self.readPackageLength = 4 + lenInd
                if self.readNum == self.readPackageLength:
                    self.getFunction()
                    self.readNum = 0
                    self.readPackageLength = 0

    def getFunction(self):
        """
        Parses a complete package, verifies CRC, and updates attributes based on function code.
        """
        ID = self.readPackage[0] & 0x7F
        ReceivedFunction_Code = self.readPackage[1] & 0x1F
        CRC_Check = (sum(self.readPackage[:self.readPackageLength - 1]) ^ self.readPackage[self.readPackageLength - 1]) & 0x7F
        if CRC_Check != 0:
            return
        funcMap = {
            isAbsolutePosition: ('motorPos32', 'MotorPosition32ReadyFlag', None),
            isMotorSpeed: ('motorSpeed32', 'MotorSpeed32ReadyFlag', None),
            isTorqueCurrent: ('motorTorqueCurrent', 'MotorTorqueCurrentReadyFlag', None),
            isStatus: ('driverStatus', 'DriveStatusFlag', 0x7F),
            isConfig: ('driverConfigByte', None, 0x7F),
            isMainGain: ('driverMainGain', None, 0x7F),
            isSpeedGain: ('driverSpeedGain', None, 0x7F),
            isIntegrationGain: ('driverIntGain', None, 0x7F),
            isTorqueConstant: ('driverTorqueConstant', None, 0x7F),
            isHighSpeed: ('driverMaxSpeed', None, 0x7F),
            isHighAccel: ('driverMaxAccel', None, 0x7F),
            isDriveID: ('driverIDNumber', None, 0x7F),
            isPosOnRange: ('driverOnRange', None, 0x7f),
            isGearNumber: ('driverGearNumber', None, None)
        }
        if ReceivedFunction_Code in funcMap:
            attr, flag, mask = funcMap[ReceivedFunction_Code]
            if flag:
                setattr(self, flag, 0x00)
            if mask is not None:
                val = self.calSignValue(self.readPackage) & mask
            else:
                val = self.calSignValue(self.readPackage) if ReceivedFunction_Code != isConfig else self.calValue(self.readPackage)
            setattr(self, attr, val)

    def calSignValue(self, onePackage):
        """
        Calculates the signed value from a package buffer by sign-extending the bits.
        """
        length = 4 + ((onePackage[1] >> 5) & 0x03)
        lcmd = onePackage[2] & 0x7F
        sign = onePackage[2] & 0x40
        for byte in onePackage[3:length-1]:
            lcmd = (lcmd << 7) | (byte & 0x7F)
        if sign:
            lcmd -= (1 << (7 * (length - 3)))
        return lcmd

    def calValue(self, onePackage):
        """
        Calculates an unsigned value from a package buffer.
        """
        length = 4 + ((onePackage[1] >> 5) & 0x03)
        lcmd = onePackage[2] & 0x7F
        for byte in onePackage[3:length-1]:
            lcmd = (lcmd << 7) | (byte & 0x7F)
        return lcmd

    @staticmethod
    def getConfigByByte(configByte):
        """Retrieves configuration description from lookup table."""
        return CONFIG_MODES.get(configByte, "Unknown Configuration")

    @staticmethod
    def getDriveConfigByNumber(index):
        """Retrieves configuration byte by index."""
        return CONFIG_REFERENCE.get(index, (None, "Invalid configuration number"))[0]

    @staticmethod
    def getStatus(byte):
        """Interpret the status byte."""
        statusList = []
        if byte & 0b00000001: statusList.append(statusFlags[0b00000001])
        if byte & 0b00000010: statusList.append(statusFlags[0b00000010])
        if byte & 0b00100000: statusList.append(statusFlags[0b00100000])
        if byte & 0b01000000: statusList.append(statusFlags[0b01000000])
        return str(statusList[-1]) if statusList else str("On Pos")

    @staticmethod
    def getAlarm(byte):
        """Extract alarm codes from bits 2-4."""
        alarmBits = (byte >> 2) & 0b111
        return alarmCodes.get(alarmBits, "Unknown Alarm")
