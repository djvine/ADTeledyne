import unittest
import epics
import time
import numpy as np

images_per_test = 3000
detector_overhead = 0.133 # seconds

acquire = epics.PV('TELE1:cam1:Acquire')
exp_time = epics.PV('TELE1:cam1:AcquireTime')
image_mode = epics.PV('TELE1:cam1:ImageMode')
array_counter = epics.PV('TELE1:cam1:ArrayCounter_RBV')
image = epics.PV('TELE1:image1:ArrayData')
num_images = epics.PV('TELE1:cam1:NumImages')
num_images_rbv = epics.PV('TELE1:cam1:NumImages_RBV')


"""
Define a successful capture as:
    - array counter increments
    - first ten elements of array are different than previously
"""

class TestShadobox(unittest.TestCase):

    def test_short_exposure_single(self):
        exp_time.put(0.025)
        time.sleep(0.1)
        image_mode.put(0) # single
        time.sleep(0.1)
        counter = array_counter.get()
        time.sleep(0.1)
        img = image.get(count=10)
        time.sleep(0.1)

        for i in range(images_per_test):
            t = time.time()
            acquire.put(1)
            while counter==array_counter.get():
                time.sleep(0.01)
                if time.time()-t>0.5:
                    raise Exception("Timeout: Failed on image: {:d}".format(i))
            counter = array_counter.get()
            self.assertTrue(~np.array_equal(img, image.get(count=10)))
        self.assertEqual(i, images_per_test-1)

    def test_long_exposure_single(self):
        exp_time.put(1.0)
        time.sleep(0.1)
        image_mode.put(0) # single
        time.sleep(0.1)
        counter = array_counter.get()
        time.sleep(0.1)
        img = image.get(count=10)
        time.sleep(0.1)

        for i in range(images_per_test):
            t = time.time()
            acquire.put(1)
            while counter==array_counter.get():
                time.sleep(0.01)
                if time.time()-t>0.5:
                    raise Exception("Timeout: Failed on image: {:d}".format(i))
            counter = array_counter.get()
            self.assertTrue(~np.array_equal(img, image.get(count=10)))
        self.assertEqual(i, images_per_test-1)

    def test_short_exposure_continuous(self):
        exp_time.put(0.025)
        time.sleep(0.1)
        image_mode.put(2) # single
        time.sleep(0.1)
        counter = array_counter.get()
        time.sleep(0.1)
        img = image.get(count=10)
        time.sleep(0.1)

        acquire.put(1)
        for i in range(images_per_test):
            t = time.time()
            while counter==array_counter.get():
                time.sleep(0.01)
                if time.time()-t>0.5:
                    raise Exception("Timeout: Failed on image: {:d}".format(i))
            counter = array_counter.get()
            self.assertTrue(~np.array_equal(img, image.get(count=10)))
        self.assertEqual(i, images_per_test-1)
        acquire.put(0)

    def test_multiple_exposure(self):
        num = 10
        exp_time.put(1.0)
        time.sleep(0.1)
        image_mode.put(1)
        time.sleep(0.1)
        num_images.put(num)
        time.sleep(0.1)

        acquire.put(1)
        time.sleep(1.0)
        while (acquire.get()!=0):
            time.sleep(1.0)
        self.assertEqual(num, num_images_rbv.get())

    def test_switching_image_modes(self):
        def take_single(n):
            exp_time.put(0.2)
            time.sleep(0.1)
            image_mode.put(0) # single
            time.sleep(0.1)
            counter = array_counter.get()
            time.sleep(0.1)
            img = image.get(count=10)
            time.sleep(0.1)

            for i in range(n):
                print('Single: {:d}'.format(i))
                t = time.time()
                acquire.put(1)
                while counter==array_counter.get():
                    time.sleep(0.01)
                    if time.time()-t>0.5:
                        raise Exception("Timeout: Failed on image: {:d}".format(i))
                counter = array_counter.get()
                self.assertTrue(~np.array_equal(img, image.get(count=10)))
            self.assertEqual(i, n-1)

        def take_multiple(n):
            exp_time.put(0.2)
            time.sleep(0.1)
            image_mode.put(1) # multiple
            time.sleep(0.1)
            num_images.put(n)
            time.sleep(0.1)
            img = image.get(count=10)
            time.sleep(0.1)

            t = time.time()
            acquire.put(1)
            time.sleep(0.1)
            while num_images_rbv.get()<n:
                print("Multiple: {:d}".format(num_images_rbv.get()))
                time.sleep(0.2)
                if time.time()-t>n*0.5:
                    raise Exception("Timeout")
            self.assertEqual(n, num_images_rbv.get())
            self.assertTrue(~np.array_equal(img, image.get(count=10)))

        def take_continuous(n):
            exp_time.put(0.2)
            time.sleep(0.1)
            image_mode.put(2) # single
            time.sleep(0.1)
            counter = array_counter.get()
            time.sleep(0.1)
            img = image.get(count=10)
            time.sleep(0.1)

            acquire.put(1)
            for i in range(n):
                print("Continuos: {:d}".format(i))
                t = time.time()
                while counter==array_counter.get():
                    time.sleep(0.01)
                    if time.time()-t>0.5:
                        raise Exception("Timeout: Failed on image: {:d}".format(i))
                counter = array_counter.get()
                self.assertTrue(~np.array_equal(img, image.get(count=10)))
            self.assertEqual(i, n-1)
            acquire.put(0)

        n=100
        for i in range(n):
            take_single(n)
            take_multiple(n)
            take_continuous(n)

if __name__=='__main__':
    unittest.main()
