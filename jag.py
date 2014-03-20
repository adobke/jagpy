import socket
import binascii
import threading
import time
import select
import pynmea2
import cmd

class Jaguar(threading.Thread):
    def __init__(self,ip):
        port = 10001
        print "Connecting to: " + ip + "..."
        
        ipAddr = map(int,ip.split('.'))
        imuIpAddr = ipAddr
        imuIpAddr[3] += 1
        imuIpAddr = map(str, imuIpAddr)
        imuIpAddr = '.'.join(imuIpAddr)

        self.logFile = open("log",'w')
        self.stop= threading.Event()

        try:
            # connect to robot control board
            self.ctlSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ctlSock.connect((ip,port))
            self.ctlSock.send("a")

            # connect to imu
            self.x_mag,self.y_mag,self.z_mag = (0,0,0)
            self.imuSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.imuSock.settimeout(2.0)
            self.imuSock.connect((imuIpAddr,10001))

            # connect to gps
            self.x_mag,self.y_mag,self.z_mag = (0,0,0)
            self.gpsSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.gpsSock.settimeout(2.0)
            self.gpsSock.connect((imuIpAddr,10002))
            self.gpsParser = pynmea2.NMEAStreamReader()
        except Exception as e:
            print "Failed to connect: " + str(e)
            self.connected = False
            return

        self.connected = True

        threading.Thread.__init__(self, name="jagpy")
        self.start()

    #def testConnection(self,ip):
    #    # Give the ip of the main robot control board
    #    # but we will try to connect to imu because it is TCP
    #    ip = map(int,ip.split('.'))
    #    ip = ipAddr
    #    ip[3] += 1
    #    ip = map(str, imuIpAddr)
    #    ip = '.'.join(imuIpAddr)
    #    try:
    #        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #        sock.settimeout(2.0)
    #        sock.connect((ip,10001))
    #    except:
    #        sock.close()
    #        return False

    #    return True

    def run(self):
        while not self.stop.isSet():
            (rSocks,_,_) = select.select([self.ctlSock, self.imuSock, self.gpsSock],[],[],1)
            for sock in rSocks:
                if sock == self.ctlSock:
                    self.dataRecieved = True
                    self.handleCtlPackets(self.ctlSock.recv(512))
                elif sock == self.imuSock:
                    self.handleImuPackets(self.imuSock.recv(512))
                elif sock == self.gpsSock:
                    self.handleGpsPackets(self.gpsSock.recv(512))

    def halt(self):
        self.go(0,0)
        self.stop.set()
        self.ctlSock.close()
        self.gpsSock.close()
        self.imuSock.close()
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
        for msg in self.gpsParser.next(packets):
            print msg

    def log(self, data):
        self.logFile.write(data+"\n")

    def send(self, comType, data):
        PACKET_START = "\x5E\x02"
        PACKET_END   = "\x5E\x0D"
        TWELVE       = "\x0C"

        crc = chr(self.crc(data))
        dest = "\x01"
        pack = PACKET_START + dest + "\x00" + comType + chr(len(data)) + data + crc + PACKET_END
        #print map(ord,pack)
        self.ctlSock.send(pack)


    def go(self,powerL,powerR):
        PWM_CTRL     = "\x06"
        MAX_PWM      = 32766
        NO_POWER     = -32768

        powerLScaled = powerL*16383.0 + 16383
        powerRScaled = (-powerR*16383.0) + 16383
        powerLScaled = max(0,min(MAX_PWM,powerLScaled))
        powerRScaled = max(0,min(MAX_PWM,powerRScaled))
        self.send(PWM_CTRL,self.encodePowers(NO_POWER,NO_POWER,NO_POWER,powerLScaled,powerRScaled,NO_POWER))
    
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

class JaguarWrapper(cmd.Cmd):
    def __init__(self,ip):
        self.robot = Jaguar(ip)
        self.prompt = ">>: "
        self.intro = "\nReady!\nEnter help for commands"
        cmd.Cmd.__init__(self)

    def do_go(self,cmd):
        cmd = map(int,cmd.split())
        try:
            self.robot.go(cmd[0],cmd[1])
        except:
            print "Invalid arguments: " + str(cmd)

    def do_go(self,cmd):
        try:
            self.robot.halt()
        except:
            print "Invalid arguments: " + str(cmd)

if __name__ == "__main__":
    r = JaguarWrapper('192.168.0.60')
