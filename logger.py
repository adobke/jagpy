import cmd
import time
from jag import Jaguar
import freenect

class JaguarWrapper(cmd.Cmd):
    def __init__(self,ip):
        self.robot = Jaguar(ip)
        self.prompt = ">>: "
        self.intro = "\nReady!\nEnter help for commands"
        cmd.Cmd.__init__(self)

    def do_go(self,cmd):
        cmd = map(float,cmd.split())
        try:
            self.robot.go(cmd[0],cmd[1])
        except:
            print "Invalid arguments: " + str(cmd)

    def do_halt(self,cmd):
        try:
            self.robot.halt()
        except:
            print "Invalid arguments: " + str(cmd)

    def do_startLog(self,cmd):
        self.robot.logAll = True

if __name__ == "__main__":
    r = JaguarWrapper('192.168.0.80')
    r.cmdloop()
    #r.go(-.4,-.4)
    #time.sleep(.1)
    #r.go(.4,.4)
    time.sleep(1)
