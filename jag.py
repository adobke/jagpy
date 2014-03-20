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
    def __init__(self,ip):
        port = 10001
        print "Connecting to: " + ip
        ipAddr = map(int,ip.split('.'))
        imuIpAddr = ipAddr
        imuIpAddr[3] += 1
        imuIpAddr = map(str, imuIpAddr)
        imuIpAddr = '.'.join(imuIpAddr)

        self.logFile = open("log",'w')
        self.stop= threading.Event()

        # connect to robot control board
        self.ctlSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctlSock.connect((ip,port))
        self.ctlSock.send("a")

        # connect to imu
        self.x_mag,self.y_mag,self.z_mag = (0,0,0)
        self.imuSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.imuSock.connect((imuIpAddr,10001))

        # connect to gps
        self.x_mag,self.y_mag,self.z_mag = (0,0,0)
        self.gpsSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.gpsSock.connect((imuIpAddr,10002))

        threading.Thread.__init__(self, name="jagpy")
        self.start()

    def run(self):
        while not self.stop.isSet():
            (rSocks,_,_) = select.select([self.ctlSock, self.imuSock, self.gpsSock],[],[],1)
            for sock in rSocks:
                if sock == self.ctlSock:
                    self.handleCtlPackets(self.ctlSock.recv(512))
                elif sock == self.imuSock:
                    self.handleImuPackets(self.imuSock.recv(512))
                elif sock == self.gpsSock:
                    self.handleGpsPackets(self.gpsSock.recv(512))

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
                (self.powerLeft,self.powerRight) = self.unpackPowers(pack[6:])[3:5]
            elif msgType == 0x7C: # encoders
                (self.encLeft, self.velLeft, self.dirLeft) = self.unpackPowers(pack[23:29])
                (self.encRight, self.velRight, self.dirRight) = self.unpackPowers(pack[29:35])
            elif msgType == 0x7d: # voltage etc
                x= 0
            else:
                print "unknown msg type " + str(msgType)
        self.log(packets)

    def handleImuPackets(self,packets):
        for packet in packets.split("#\r\n"):
            packet = packet.lstrip()
            if len(packet) == 0:
                continue
            if packet[0] != "$":
                continue
            packet = packet[1:]
            try:
                packet = map(int,packet.split(","))
            except:
                continue
            self.x_accel = packet[1]
            self.y_accel = packet[2]
            self.z_accel = packet[3]
            self.x_gyro = packet[4]
            self.y_gyro = packet[5]
            self.z_gyro = packet[6]
            if packet[9] != 0:
                self.x_mag = packet[7]
                self.y_mag = packet[8]
                self.z_mag = packet[9]
            #print self.x_accel,self.y_accel,self.z_accel
            #print self.x_mag,self.y_mag,self.z_mag
            
    def handleGpsPackets(self,packets):
        print packets

    def log(self, data):
        self.logFile.write(data+"\n")

    def send(self, comType, data):
        crc = chr(self.crc(data))
        dest = "\x01"
        pack = PACKET_START + dest + "\x00" + comType + chr(len(data)) + data + crc + PACKET_END
        #print map(ord,pack)
        self.ctlSock.send(pack)


    def go(self,powerL,powerR):
        powerLScaled = powerL*16383.0 + 16383
        powerRScaled = (-powerR*16383.0) + 16383
        noPower = -32768
        self.send(PWM_CTRL,self.encodePowers(noPower,noPower,noPower,powerLScaled,powerRScaled,noPower))
    
    @staticmethod
    def unpackPowers(powers):
        values = []
        for (low,high) in zip(powers[::2],powers[1::2]):
            values.append(low + (high << 8))
        return values

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
    r = Jaguar('192.168.0.60') 
    time.sleep(1)
    r.go(0,.1)
    time.sleep(1)
    r.go(0,.2)
    time.sleep(1)
    r.go(0,.3)
    time.sleep(1)
    r.go(0,.4)
    time.sleep(1)
    #r.go(0,0)
