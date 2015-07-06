"""
Creates an instance of a file loader. Takes in a file and parses it into a stream of experience
where each step is a dictionary in an array. The dictionary keys are as specified in the header of
the file.
"""
__author__ = 'alex'


class FileLoader(object):

    def __init__(self, file_loc):
        """Initializes a file-loader given a file-location"""
        f = open(file_loc, 'r')                 # open the file for reading
        self.data_stream = []                   # this is where we store the stream of experience
        self.elements = f.readline().rstrip().split(',')  # extracts the header of the file

        for line in f:
            vals = line.rstrip().split(',')              # separate all the values in an observation
            self.data_stream.append(dict([(self.elements[i], float(vals[i])) for i in range(len(vals)-1)]))
            # we -1 because there's another comma at the end of every line but the header
            # todo: generalize this to work with cleaned obs
        self.i = 0

    def has_obs(self):
        return len(self.data_stream) > self.i

    def step(self):
        """If there are still observations, returns the next observation. Else returns None"""
        if self.has_obs():           # while we still have experience
            self.i += 1                             # move to the nexr obs
            return self.data_stream[self.i-1]         # return the next observation
        else:                                   # otherwise we have nothing left
            return None                         # so return None.


class File_Loader_Approx(object):

    def __init__(self, file_loc, n):
        """Initializes a file-loader given a file-location, only takes every 14th step"""
        f = open(file_loc, 'r')                 # open the file for reading
        self.data_stream = []                   # this is where we store the stream of experience
        self.elements = f.readline().rstrip().split(',')  # extracts the header of the file
        i = 0
        for line in f:
            if i % n == 0:
                vals = line.rstrip().split(',')              # separate all the values in an observation
                self.data_stream.append(dict([(self.elements[i], float(vals[i])) for i in range(len(vals)-1)]))
                # we -1 because there's another comma at the end of every line but the header
                # todo: generalize this to work with cleaned obs
            i += 1
        self.i = 0

    def has_obs(self):
        return len(self.data_stream) > self.i

    def step(self):
        """If there are still observations, returns the next observation. Else returns None"""
        if self.has_obs():           # while we still have experience
            self.i += 1                             # move to the nexr obs
            return self.data_stream[self.i-1]         # return the next observation
        else:                                   # otherwise we have nothing left
            return None                         # so return None.