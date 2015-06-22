"""
    Test the FileLoader
"""
import unittest

from pysrc.utilities.file_loader import FileLoader

__author__ = 'alex'

class IsOddTests(unittest.TestCase):

    def testOne(self):
        """ Uses a small one-observation test file. """
        self.fl = FileLoader('test1.txt')

        header = ['a', 'b', 'c', 'd', 'e', 'f', 'g']                # construct actual header
        self.assertEquals(self.fl.elements, header)                 # check parsed header
        self.assertEquals(len(self.fl.data_stream), 1)              # compare data-stream size with actual
        obs = self.fl.step()                                        # get the first obs
        truth = dict([(header[i], i) for i in range(len(header))])  # construct the actual obs
        for k in truth:                                             # for all keys in an obs
            self.assertEquals(obs[k], truth[k])                     # ... check compare with truth

def main():
    unittest.main()

if __name__ == '__main__':
    main()