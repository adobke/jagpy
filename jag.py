import socket
import threading
import select
import pynmea2


PACKET_END   = "\x5E\x0D"

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
        self.logAll = False;
        self.stop= threading.Event()

        # connect to robot control board
        self.dataRecieved = False
        self.ctlSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctlSock.connect((ip,port))
        self.ctlSock.send("a")

        try:
            # connect to robot control board
            self.laserSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.laserSock.settimeout(2.0)
            self.laserSock.connect((ip,10002))
            self.laserSock.send("BM\x0D")
            self.laserSock.send("GD0045072503\x0D")
        except Exception as e:
            print "Failed to connect to laser: " + str(e)
            self.connected = False
            return

        try:
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
    #    # but we will try to connect to imu because it is TCP #    ip = map(int,ip.split('.')) #    ip = ipAddr #    ip[3] += 1 #    ip = map(str, imuIpAddr)
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
            (rSocks,_,_) = select.select([self.ctlSock, self.laserSock, self.imuSock, self.gpsSock],[],[],1)
            for sock in rSocks:
                if sock == self.ctlSock:
                    if not self.dataRecieved:
                        print "Connected"
                    self.dataRecieved = True
                    self.handleCtlPackets(self.ctlSock.recv(512))
                elif sock == self.laserSock:
                    self.handleLaserPackets(self.laserSock.recv(512))
                    self.laserSock.send("GD0045072503\x0D")
                elif sock == self.imuSock:
                    self.handleImuPackets(self.imuSock.recv(512))
                elif sock == self.gpsSock:
                    self.handleGpsPackets(self.gpsSock.recv(512))

    def halt(self):
        self.go(0,0)
        self.stop.set()
        self.logAll = False
        self.ctlSock.close()
        self.gpsSock.close()
        self.imuSock.close()
        self.logFile.close()
        self.join()

    def handleLaserPackets(self,packets):
        for packet in packets.split('\x0D'):
            if "00P" in packet:
                distances = []
                packet = packet.split('\n')
                #print packet
                packet = reduce(lambda x,y: x+y[:-1], packet[3:])
                #print len(packet)
                for i in range(0,len(packet)-1,2):
                    pack = packet[i:i+2]
                    d = 0
                    for c in pack:
                        d = d << 6
                        d = d + ord(c) - 0x30
                    distances.append(d)    
                    #distances.append( ((ord(pack[0]) - 0x30) << 6) | (ord(pack[1])- 0x30) )
                    #distances.append( ((ord(pack[0]) - 0x30) << 12) | ((ord(pack[1]) - 0x30) << 6) | (ord(pack[2])- 0x30) )
                    #if len(pack) is 2:
                    #    distances.append( ((ord(pack[0]) - 0x30) << 6) | (ord(pack[1])- 0x30) )
                    #    #distances.append( ((ord(pack[0]) - 0x30) << 16 | (ord(pack[1]) - 0x30) << 12) | ((ord(pack[2]) - 0x30) << 6) | (ord(pack[3])- 0x30) )
                self.distances = distances

    def handleCtlPackets(self,packets):
        for packet in packets.split(PACKET_END):
            pack = map(ord,packet)
            if pack == []:
                continue
            msgType = pack[4]

            if msgType == 0xFF: # system msg
                if pack[6] == 1: # we need to ack these
                    self.send('\xFF',"\x01")
            elif msgType == 0x7B: # motor snsr
                (self.powerLeft,self.powerRight) = self.unpackPowers(pack[6:])[3:5]
            elif msgType == 0x7C: # encoders
                (self.encLeft, self.velLeft, self.dirLeft) = self.unpackPowers(pack[23:29])
                (self.encRight, self.velRight, self.dirRight) = self.unpackPowers(pack[29:35])
                self.log("odo,{0},{1}".format(self.encLeft,self.encRight))
            elif msgType == 0x7d: # voltage etc
                x= 0
            else:
                print "unknown msg type " + str(msgType)

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
            
            if len(packet) < 10:
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
            self.log("imu,{0},{1},{2},{3},{3},{4},{5},{6},{7},{8}".format(self.x_accel,self.y_accel,self.z_accel\
                    ,self.x_gyro,self.y_gyro,self.z_gyro,self.x_mag,self.y_mag,self.z_mag))
            
    def handleGpsPackets(self,packets):
        try:
            for msg in self.gpsParser.next(packets):
                if type(msg) is pynmea2.types.talker.GGA:
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
                    self.gps_time = msg.timestamp
                    self.gps_qual = msg.gps_qual
                    self.log("gps,{0},{1},{2}".format(self.latitude,self.longitude,self.gps_qual))
                elif type(msg) is pynmea2.types.talker.RMC:
                    continue
                else:
                    continue
        except:
            x = 0


    def log(self, data):
        if(self.logAll):
            print "should log"
            self.logFile.write(str(self.gps_time)+","+data+"\n")

    def send(self, comType, data):
        PACKET_START = "\x5E\x02"
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

