import json
import paho.mqtt.client as mqtt
import serial
import time
import sys
import traceback
from io import StringIO

SERIAL_PORT    = '/dev/ttyAMA0'   #RPi
#SERIAL_PORT    = 'COM4'            #Windows
SERIAL_BAUD    = 9600
SERIAL_TIMEOUT = 1

MQTT_HOST       = '10.0.0.200'
MQTT_TCP_PORT   = 1883
MQTT_KEEPALIVE  = 60
MQTT_CLIENT_ID  = 'RPi_Ninja_BG'
MQTT_TOPIC_BASE = 'rpi'

UTF_8           = 'utf-8'
NEW_LINE        = '\r\n'

MSG_TYPE_DEVICE = 'DEVICE'
MSG_TYPE_ACK    = 'ACK'
MSG_TYPE_ERROR  = 'ERROR'

GUID_CODE   = 'G'
VENDOR_CODE = 'V'
DEV_CODE    = 'D'
DATA_CODE   = 'DA'

GUID_ID   = '0'
GUID_TEMP_ID = '0101'
VENDOR_ID = 0

DEV_433MHZ_ID   = 11
DEV_HUMIDITY_ID = 30
DEV_TEMP_ID     = 31
DEV_STATUS_ID   = 999
DEV_VERSION_ID  = 1003
DEV_EYES_ID     = 1007

DATA_STATUS_BLACK   = '000000'  #
DATA_STATUS_RED     = 'FF0000'  #                               Me: Process failed.
DATA_STATUS_YELLOW  = 'FFFF00'  #                               Me: No ACK from shield.
DATA_STATUS_GREEN   = '00FF00'  # All good.                     Me: All good.
DATA_STATUS_CYAN    = '00FFFF'  # Not connected to network.     Me: Error message from shield.
DATA_STATUS_BLUE    = '0000FF'  # Cannot connect to shield.     Me: Cannot connect to shield.
DATA_STATUS_MAGENTA = 'FF00FF'  # Waiting to be paired.         Me: Invalid message received from shield.
DATA_STATUS_WHITE   = 'FFFFFF'  #                               Me: Cannot connect to network.

DATA_STATUS_TEXT = {DATA_STATUS_BLACK: 'Status Unknown',
                    DATA_STATUS_RED: 'DISCONNECTED',
                    DATA_STATUS_YELLOW: 'MODULE UNRESPONSIVE',
                    DATA_STATUS_GREEN: 'ONLINE',
                    DATA_STATUS_CYAN: 'MODULE ERROR',
                    DATA_STATUS_BLUE: 'OFFLINE',
                    DATA_STATUS_MAGENTA: 'MODULE GARBLED',
                    DATA_STATUS_WHITE: 'COULD NOT CONNECT'}

TIMEOUT_FOR_SHIELD_ACK = 1.0    # Time until no ack means error.


#region Global Variables
statusLED = DATA_STATUS_BLUE

isConnectedToNetwork = False
isConnectedToShield  = False
lastSerialMessageSent     = time.time()
lastSerialMessageReceived = time.time()
lastSerialMessageAcked    = time.time()
lastSerialMessageError    = time.time()
lastSerialMessageInvalid  = time.time()

lastMqttMessageReceived   = time.time()
mqttLastTopicSent         = ''
mqttLastMessageSent       = ''
mqttTimeLastMessageSent   = time.time()

arduinoVersion = 'Unknown'
#endregion

def checkStatusLED():
    global statusLED

    desiredColour = DATA_STATUS_BLACK
    if(not isConnectedToShield or time.time() - lastSerialMessageReceived > 62.0):
        desiredColour = DATA_STATUS_BLUE
    elif(not isConnectedToNetwork):
        desiredColour = DATA_STATUS_WHITE
    elif(lastSerialMessageAcked < lastSerialMessageSent and time.time() - lastSerialMessageSent > 1.0):
        desiredColour = DATA_STATUS_YELLOW
    elif(lastSerialMessageError > lastSerialMessageAcked and lastSerialMessageError > lastMqttMessageReceived):
        desiredColour = DATA_STATUS_CYAN
    elif(lastSerialMessageInvalid > lastSerialMessageAcked and lastSerialMessageInvalid > lastSerialMessageReceived):
        desiredColour = DATA_STATUS_MAGENTA
    else:
        desiredColour = DATA_STATUS_GREEN

    sendStatus(desiredColour)

def sendStatus(newColour):
    global statusLED
    if(newColour != statusLED):
        sendSerialCommand(MSG_TYPE_DEVICE, GUID_ID, VENDOR_ID, DEV_STATUS_ID, 0, 0.0, newColour, True, False)
        try:
            if(isConnectedToNetwork):
                client.publish(MQTT_TOPIC_BASE + '/' + 'status', DATA_STATUS_TEXT[newColour], qos=1, retain=True)
        except:
            print('Problem sending message on network.')
        statusLED = newColour

def getVersion():
    global versionString

    try:
        sendSerialCommand(MSG_TYPE_DEVICE, GUID_ID, VENDOR_ID, DEV_VERSION_ID, 0, 0.0, 'VNO', True, False)
    except:
        pass



#MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print('Connected with result code ' + str(rc))
    client.subscribe(MQTT_TOPIC_BASE + '/#')
    global isConnectedToNetwork
    isConnectedToNetwork = True

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print('Unexpected disconnect with result code ' + str(rc))
    global isConnectedToNetwork
    isConnectedToNetwork = False



def on_message(client, userdata, msg):
    now = time.time()
    if(now - mqttTimeLastMessageSent > 2.0 or mqttLastTopicSent != msg.topic or mqttLastMessageSent != (msg.payload).decode(UTF_8)):
        print("RX MQTT: " + msg.topic + ": " + (msg.payload).decode(UTF_8))
        receiveMQTT(msg.topic, msg.payload)



#RECEIVE MQTT   mqtt_topic is of type str, data is of type bytes
def receiveMQTT(mqtt_topic, data):
    noError = True
    ids = mqtt_topic.split('/')
    if(len(ids) == 3 and ids[0] == MQTT_TOPIC_BASE):
        try:
            vendorID = int(ids[1])
            deviceID = int(ids[2])
            datastr  = data.decode(UTF_8)
            # Only allow known values.
            if(vendorID == VENDOR_ID and (deviceID == DEV_433MHZ_ID or deviceID == DEV_EYES_ID)):
                sendSerialCommand(MSG_TYPE_DEVICE, GUID_ID, vendorID, deviceID, 0, 0.0, datastr, True, False)
            else:
                print('Invalid vendor ID: ' + str(vendorID) + ', or device ID: ' + str(deviceID))
        except ValueError:
            print('Unrecognised topic: ' + mqtt_topic)
        except UnicodeError:
            print('Unrecognised characters in data')
    else:
        print('Unrecognised topic: ' + mqtt_topic)


#RECEIVE SERIAL
def receiveSerial():
    if(ser.in_waiting > 0):
        line = ser.readline()
        strLine = str(line, UTF_8)
        if(len(strLine) > 0):
            print("RX SERIAL : " + strLine.strip('\r\n'))
            decodeSerial(strLine)


def decodeSerial(data):
    global lastSerialMessageError
    global lastSerialMessageAcked
    global lastSerialMessageReceived
    global arduinoVersion

    dict = None
    try:
        dict = json.loads(data)
    except JSONDecodeError:
        print('Invalid JSON received from shield.')
        lastSerialMessageError = time.time()
    except NameError:
        print('Invalid JSON Receivd from shield.')
        lastSerialMessageError = time.time()
    except TypeError:
        print('Invalid JSON received from shield.')
        lastSerialMessageError = time.time()
    else:
        if MSG_TYPE_ACK in dict:
            lastSerialMessageAcked = time.time()
            ids = dict[MSG_TYPE_ACK]
            try:
                if len(ids) == 1:
                    guid = ids[0][GUID_CODE]
                    vid  = ids[0][VENDOR_CODE]
                    did  = ids[0][DEV_CODE]
                    da   = ids[0][DATA_CODE]
            except:
                print('Invalid ACK format')
            else:
                if(type(did) is int and did == DEV_VERSION_ID):
                    if(type(da) is str):
                        print('Version is: ' + da)
                        arduinoVersion = da


        elif MSG_TYPE_ERROR in dict:
            lastSerialMessageError = time.time()
            ids = dict[MSG_TYPE_ERROR]
            try:
                if len(ids) == 1:
                    errorCode = ids[0]['CODE']
            except KeyError:
                print('Unknown error from shield.')
            except TypeError:
                print('Unknown error from shield.')
            else:
                print('Error response from shield: ' + str(errorCode))

        elif MSG_TYPE_DEVICE in dict:
            try:
                adjustForRfHexToBinary(dict, MSG_TYPE_DEVICE)
            except:
                pass

            ids = dict[MSG_TYPE_DEVICE]
            try:
                if len(ids) == 1:
                    guid = ids[0][GUID_CODE]
                    vid  = ids[0][VENDOR_CODE]
                    did  = ids[0][DEV_CODE]
                    da   = ids[0][DATA_CODE]
                else:
                    raise TypeError
            except KeyError:
                print('Could not find desired keys in JSON result')
                lastSerialMessageError = time.time()
            except TypeError:
                print('Could not find desired keys in JSON result')
                lastSerialMessageError = time.time()
            else:
                if(type(da) is str):
                    lastSerialMessageReceived = time.time()
                    processSerialDeviceResult(guid,vid,did,da,0,0.0,True,False)
                elif(type(da) is float):
                    lastSerialMessageReceived = time.time()
                    processSerialDeviceResult(guid,vid,did,'',0,da,False,True)
                elif(type(da) is int):
                    lastSerialMessageReceived = time.time()
                    processSerialDeviceResult(guid,vid,did,'',da,0.0,False,False)


def processSerialDeviceResult(gid, vid, did, strData, intData, floatData, isDataString, isDataFloat):
    global statusLED
    global lastSerialMessageReceived
    if(gid == GUID_ID):
        if(did == DEV_433MHZ_ID and vid >= 0 and isDataString):
            publishMQTT(vid, did, strData, intData, floatData, isDataString, isDataFloat)
        elif(did == DEV_EYES_ID and isDataString):
            publishMQTT(vid, did, strData, intData, floatData, isDataString, isDataFloat)
        elif(did == DEV_STATUS_ID and vid == VENDOR_ID and isDataString):
            statusLED = strData
    elif(gid == GUID_TEMP_ID):
        if(did == DEV_TEMP_ID and vid >= 0):
            publishMQTT(vid, did, strData, intData, floatData, isDataString, isDataFloat)
        elif(did == DEV_HUMIDITY_ID and vid >= 0):
            publishMQTT(vid, did, strData, intData, floatData, isDataString, isDataFloat)

def publishMQTT(vid, did, strData, intData, floatData, isDataString, isDataFloat):
    global mqttLastTopicSent
    global mqttLastMessageSent
    global mqttTimeLastMessageSent
    strTopic = MQTT_TOPIC_BASE + '/' + str(vid) + '/' + str(did)
    if isDataString:
        strDataConv = strData
    elif isDataFloat:
        strDataConv = str(floatData)
    else:
        strDataConv = str(intData)
    mqttLastTopicSent = strTopic
    mqttLastMessageSent = strDataConv
    mqttTimeLastMessageSent = time.time()
    print("TX MQTT: " + strTopic + ": " + strDataConv)
    if(did == DEV_433MHZ_ID):
        client.publish(strTopic, strDataConv, qos=1, retain=False)
    else:
        client.publish(strTopic, strDataConv, qos=1, retain=True)


#SEND SERIAL COMMAND
def sendSerialCommand(msgType, gID, venID, devID, intData, floatData, strData, isDataString, isDataFloat):
    global isConnectedToShield
    global lastSerialMessageSent
    dict = None
    if(msgType == MSG_TYPE_ERROR):
        dict = {MSG_TYPE_ERROR: [{'CODE': intData}]}
    elif(msgType == MSG_TYPE_DEVICE or msgType == MSG_TYPE_ACK):
        if(len(gID) > 0 and venID >= 0 and devID >= 0):
            if(isDataString and strData != None and len(strData) > 0):
                if(strData != None and len(strData) > 0):
                    dict = {msgType: [{GUID_CODE:gID, VENDOR_CODE:venID, DEV_CODE:devID, DATA_CODE: strData}]}
                    adjustForRfHexToBinary(dict, msgType)
                else:
                    print('No data string supplied')
            elif(isDataFloat):
                dict = {msgType: [{GUID_CODE:gID, VENDOR_CODE:venID, DEV_CODE:devID, DATA_CODE: floatData}]}
            else:
                dict = {msgType: [{GUID_CODE:gID, VENDOR_CODE:venID, DEV_CODE:devID, DATA_CODE: intData}]}
        else:
            print('Invalid gID: ' + giD + ', venID: ' + str(venID) + ' or devID: ' +  str(devID))
    else:
        print('Unknown message type')
    result = json.dumps(dict, separators=(',',':')) + NEW_LINE

    if(ser.isOpen()):
        try:
            print("TX SERIAL: " + result.strip('\r\n'))
            ser.write(bytes(result, UTF_8))
        except:
            print('Unknown error sending Serial command')
        else:
            lastSerialMessageSent = time.time()
            isConnectedToShield = True
    else:
        print('Could not communicate with serial port.')
        isConnectedToShield = False

def adjustForRfHexToBinary(dict, msgType):
    if(dict[msgType][0][DEV_CODE] == DEV_433MHZ_ID):
        strData = dict[msgType][0][DATA_CODE]
        if(type(strData) is str):
            if(len(strData) <= 8): #If 0-8 bytes in length, must be hex and needs to be converted to binary.
                try:
                    intData = int(strData, 16)
                    strData = '{0:024b}'.format(intData)
                except ValueError:
                    print('Invalid data value for ' + DEV_433MHZ_ID + ' device ID')
            elif(len(strData) >= 16): #If 16 or more bytes in length, must be binary and needs to be converted to hex.
                try:
                    intData = int(strData, 2)
                    strData = '{0:06X}'.format(intData)
                except ValueError:
                    print('Invalid data value for ' + DEV_433MHZ_ID + ' device ID')
            dict[msgType][0][DATA_CODE] = strData
        else:
            print('Not string data')


#region Main Method
#MAIN METHOD
time.sleep(1) #This script is normally run at startup, so this will give the network time to get up.
try:
    ser = serial.Serial(SERIAL_PORT, timeout=0.5)
    isConnectedToShield = True
    getVersion()
except:
    print("Something went wrong connecting to the serial port")
    traceback.print_exc(file=sys.stdout)

try:
    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True, userdata=None)
    client.will_set(MQTT_TOPIC_BASE + '/' + 'status', 'DISCONNECTED', qos=1, retain=True)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(MQTT_HOST, MQTT_TCP_PORT, MQTT_KEEPALIVE)

    client.loop_start()

    isConnectedToNetwork = True

except:
    print("Something went wrong connecting to the mqtt server.")
    traceback.print_exc(file=sys.stdout)

try:
    while True:
        receiveSerial()
        time.sleep(0.1)
        checkStatusLED()
except:
    print("Unexpected exception")
    traceback.print_exc(file=sys.stdout)
#    print("Unexpected error:" + str(sys.exc_info()[0]) + ', ' + str(sys.exc_info()[1]))
#    print(sys.exc_info()[2])

sendStatus(DATA_STATUS_RED)
client.loop_stop()
client.disconnect()
ser.close()

#endregion
