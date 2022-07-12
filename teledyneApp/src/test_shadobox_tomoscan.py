import unittest
import tomoscan as ts

def setUp

class TestShadobox(unittest.TestCase):

    def setUp(self):
        pass

    def test_short_exposure(self):
        """
        3601 images at 25 ms
        """
        pass

    def test_long_exposure(self):
        """
        3601 images at 0.2 seconds
        """
        pass

    def test_multiscan(self):
        """
        3601 imags at 50 ms
        continuous mode for 100 images
        3601 images at 50 ms
        """
        pass

if __name__=='__main__':
    unittest.main()



