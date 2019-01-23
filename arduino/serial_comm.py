import serial
import time
import threading
#import sys

START = '@'.encode()
END   = '#'.encode()
UNIT  = "02".decode("hex")
ACK   = "00".decode("hex")
START_COMM = "00".decode("hex")
SPEED_0 = "00".decode("hex")

COMM_STATE = "IDLE"
SEND_ONES = 0

"""
PROTOCOL
  ----------------
  0 - Start byte
    0x40 (@) = Start
  1 - Sender/Receiver
    0x01 = Arduino
    0x02 = PI/PC
  2 - Command/Info
    0x00 = START COMM
    0x01 = ACK
    0x02 = NACK
    0x03 = SET SPEED // Set speed in next byte (3 - Speed)
    0x04 = POS // Indicate pos. Used on Arduino side, otherwise 0
    0x05 = CHANGE LINE
    0x06 = FOLLOW LINE
  3 - Speed
    0x00 - 0xFF = SPEED //Only set on Pi side, otherwise 0
  4 - POS
    0x00 - 0xFF = X1
    0x00 - 0xFF = X2
    0x00 - 0xFF = X3
    0x00 - 0xFF = X4

    0x00 - 0xFF = Y1
    0x00 - 0xFF = Y2
    0x00 - 0xFF = Y3
    0x00 - 0xFF = Y4

    0x00 - 0xFF = A1
    0x00 - 0xFF = A2
    0x00 - 0xFF = A3
    0x00 - 0xFF = A4
  5 - END
    0x23 (#) = END BYTE

    //===================
    0 - Start byte
    1 - Sender/Receiver
    2 - Command/Info
    3 - Speed //Used on Pi side, otherwise 0x00
    4 - POS //Used on Arduino side, otherwise 0x00
    5 - END
"""

def stringToHEX(array):
	return ":".join("{:02x}".format(ord(c)) for c in array)

def stringToHEX2(array):
	hexarray = ""
	for element in array:
		#print(type(element))
		hexarray = hexarray + str(hex(element)) + ":"
	return hexarray


def createProt(cmd=ACK, speed=SPEED_0):

	global START
	global END
	global UNIT

	prot = bytearray([START, UNIT, cmd, speed, END])
	return prot

def ctrl():

	global COMM_STATE
	global SEND_ONES
	MAX_MESSAGE_LEN = 16
	recv_message = b''

	#fredrik
	ser = serial.Serial('/dev/tty.usbserial-A506BOJ1', 9600)
	#oskar
	#ser = serial.Serial('/dev/tty.usbserial-A5050QDI', 9600)

	while(True):

		size = ser.inWaiting()
		tmp_message = ser.read(size)
		print("Recv : " + stringToHEX(tmp_message) + " | size : " + str(size))
		recv_message = recv_message + tmp_message

		if len(recv_message) > MAX_MESSAGE_LEN:
			print("Clear buffer")
			recv_message = b''

		# STATE 1 : IDLE STATE
		if COMM_STATE == "IDLE":

			if SEND_ONES == 0:
				#Start communcation

				sendMsg = createProt(START_COMM)
				ser.write(sendMsg)
				print("Send : start comm : " + stringToHEX2(sendMsg))
				SEND_ONES = 1
				time.sleep(1)

			if len(recv_message) == 6 and recv_message[0] == START and recv_message[5] == END and recv_message[2] == ACK:
				COMM_STATE = "BEGIN_COMM"
			else:
				SEND_ONES = 0

		# STATE 2 : LIVE COMMUNCATION
		elif COMM_STATE == "BEGIN_COMM":
			print('HERE')

		time.sleep(1)
	ser.close()

#t1 = threading.Thread(target=keyboard_reader)
#t1.start()

t2 = threading.Thread(target=ctrl)
t2.start()



"""
def keyboard_reader():
	global speed_changed
	global speed

	while(True):
		key = raw_input("Select speed, fast/slow/stop/stop keyboard reader (f/s/q/c)")
		print key

		if key == str('f'):
			speed_changed = 1
			speed = 100
		elif key == str('s'):
			speed_changed = 1
			speed = 70
		elif key == str('q'):
			speed_changed = 1
			speed = 0
		elif key == str('c'):
			sys.exit()

		print "Speed changed : " + str(speed_changed) + " to speed : " + str(speed)

		time.sleep(0.1)
"""
