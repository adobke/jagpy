import socket
import binascii
import threading
import time
import select

PACKET_START = "\x5E\x02"
PACKET_END   = "\x5E\x0D"
PWM_CTRL     = "\x06"
TWELVE       = "\x0C"

class Jaguar(threading.Thread):
    def __init__(self,ip,port):
        self.logFile = open("log",'w')
        self.stop= threading.Event()

        self.ctlSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctlSock.connect((ip,port))
        self.ctlSock.send("a")

        threading.Thread.__init__(self, name="jagpy")
        self.start()

    def run(self):
        print "starting"
        while not self.stop.isSet():
            (rSocks,_,_) = select.select([self.ctlSock],[],[],1)
            for sock in rSocks:
                if sock == self.ctlSock:
                    self.handleCtlPackets(self.ctlSock.recv(512))

    def halt(self):
        self.stop.set()
        self.ctlSock.close()
        self.logFile.close()
        self.join()

    def handleCtlPackets(self,packets):
        for packet in packets.split(PACKET_END):
            pack = map(ord,packet)
            if pack == []:
                continue
            msgType = pack[4]

            if msgType == 0xFF: # system msg
                print "System msg: " + str(pack[6])
                if pack[6] == 1: # we need to ack these
                    self.send('\xFF',"\x01")
            elif msgType == 0x7B: # motor snsr
                print "motor sensor: " + str(pack[6:])
            elif msgType == 0x7C: # motor snsr
                print "custom sensor: " + str(pack[6:])
            elif msgType == 0x7d: # motor snsr
                print "standard sensor: " + str(pack[6:])
            else:
                print "unknown msg type " + str(msgType)
        self.log(packets)

    def log(self, data):
        self.logFile.write(data+"\n")

    def send(self, comType, data):
        crc = chr(self.crc(data))
        dest = "\x01"
        pack = PACKET_START + dest + "\x00" + comType + chr(len(data)) + data + crc + PACKET_END
        print map(ord,pack)
        self.ctlSock.send(pack)
        print ">>>> " + pack


    def go(self,powerL,powerR):
        powerLScaled = powerL*16383.0 + 16383
        powerRScaled = (-powerR*16383.0) + 16383
        noPower = -32768
        self.send(PWM_CTRL,self.encodePowers(noPower,noPower,noPower,powerLScaled,powerRScaled,noPower))
    
    @staticmethod
    def encodePowers(*args):
        args = map(int,args)
        powerStr = ""
        for power in args:
            powerStr += chr(power & 0xFF)
            powerStr += chr((power >> 8) & 0xFF)
        return powerStr

    @staticmethod
    def crc(data):
        shift_reg = 0
        for c in map(ord,data):
            for j in range(8):
                bit = c & 0x01
                sr  = shift_reg & 0x01
                fb  = (bit^sr) & 0x01
                shift_reg = shift_reg >> 1
                if (fb == 1):
                    shift_reg = shift_reg ^ 0x8C
                c = c >> 1
        return shift_reg

if __name__ == "__main__":
    r = Jaguar('192.168.0.60',10001) 
    time.sleep(1)
    r.go(.1,.2)
    time.sleep(1)
    r.go(.3,.2)
    time.sleep(1)
    r.go(.5,.2)
    time.sleep(1)
    r.go(1,.2)
    time.sleep(1)
    #r.go(0,0)
