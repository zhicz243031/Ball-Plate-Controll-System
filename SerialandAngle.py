import serial.tools.list_ports
import time
from struct import pack,unpack

# open the serial port COM4
ser = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)

# Test the serial port
def DetectSerPort():
    plist = list(serial.tools.list_ports.comports())
    print(plist[0])
    if len(plist) <= 0:
        print("没有发现端口!")
    else:
        plist_0 = list(plist[0])
        serialName = plist_0[0]
        serialFd = serial.Serial(serialName, 9600, timeout=60)
        print("可用端口名>>>", serialFd.name)

# convert the float types into hex type
def flo2hex(alpha, beta):

    bytealpha=pack('f',alpha)
    byteAlphaReverse = bytealpha[::-1]

    bytebeta=pack('f',beta)
    byteBetaReverse = bytebeta[::-1]

    return byteAlphaReverse,byteBetaReverse

# combine the different strings
def Angle2SerPort(alpha,beta):

    BeginStr = b'>*>'
    EndStr = b'<*<'
    CommaStr = b','

    hexAlpha,hexBeta=flo2hex(alpha, beta)

    CommondStr = BeginStr+hexAlpha+CommaStr+hexBeta+EndStr
    Sendmessagelong = ser.write(CommondStr)


Angle2SerPort(0,0)
# DetectSerPort()
# ser.close()