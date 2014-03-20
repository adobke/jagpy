import jag
import unittest

class BasicTests(unittest.TestCase):
    def test_crc0(self):
        arr = [1,0,27,12,0,0,0,0,0,0,0,0,0,0,0,0]
        arrStr = ''.join(map(chr,arr))
        self.assertEqual( jag.Jaguar.crc(arrStr), 152 )
        
    def test_crc1(self):
        arr = [1,0,27,12,0,0,0,0,0,0,2,0,254,255,0,0]
        arrStr = ''.join(map(chr,arr))
        self.assertEqual( jag.Jaguar.crc(arrStr), 64 )

    def test_motorpowerenc1(self):
        self.assertEqual( jag.Jaguar.encodePowers(0,0,0,16000,16000,0), \
                "\x00\x00\x00\x00\x00\x00\x80\x3e\x80\x3e\x00\x00" )
    def test_motorpowerenc2(self):
        self.assertEqual( jag.Jaguar.encodePowers(0,0,0,17000,17000,0), \
                "\x00\x00\x00\x00\x00\x00\x68\x42\x68\x42\x00\x00" )
    def test_motorpowerenc3(self):
        self.assertEqual( jag.Jaguar.encodePowers(0,0,0,18000,11000,0), \
                "\x00\x00\x00\x00\x00\x00\x50\x46\xf8\x2a\x00\x00" )

if __name__ == "__main__":
    unittest.main()

