"""
Creates an instance of a file loader. Takes in a file and parses it into a stream of experience
where each step is a dictionary in an array. The dictionary keys are as specified in the header of
the file.
"""
__author__ = 'alex'


class Max_Min_Finder(object):

    def __init__(self, file_loc):
        """Initializes a file-loader given a file-location"""
        f = open(file_loc, 'r')                 # open the file for reading
        self.data_stream = []                   # this is where we store the stream of experience
        self.elements = f.readline().rstrip().split(',')  # extracts the header of the file

        for line in f:
            vals = line.rstrip().split(',')              # separate all the values in an observation
            self.data_stream.append(dict([(self.elements[i], float(vals[i])) for i in range(len(vals)-1)]))
            # we -1 because there's another comma at the end of every line but the header
        self.i = 0

        self.max_v = dict([(k, 0) for k in self.elements])
        self.min_v = dict([(k, 0) for k in self.elements])

        for state in self.data_stream:
            for e in self.elements:
                if state[e] > self.max_v[e]:
                    self.max_v[e] = state[e]
                if state[e] < self.min_v[e]:
                    self.min_v[e] = state[e]

        for e in self.elements:
            print('{e}: max: {max}, min: {min}'.format(e=e, max=self.max_v[e], min=self.min_v[e]))


def main():
    print('starting')
    file = 'results/prosthetic-data/EdwardsPOIswitching_s1a1.txt'
    m = Max_Min_Finder(file)


def generate_normalizer(datastream, prob):
    """ given a problem and a data stream finds the values to normalize the feature-vectors """
    for obs in datastream:
        state = prob.get_state(obs)

        try:                                    # Check if the current state is higher or lower than max/min
            for i in range(len(state)):
                (high, low) = normalizer[i]
                if state[i] > high:
                    high = state[i]
                if state[i] < low:
                    low = state[i]
                normalizer[i] = (high, low)     # Re-assign vals
        except:                                 # if we haven't made a max/min, instantiate
            normalizer = [(val, val) for val in state]
    return normalizer


def find_invalid(state, obs):
    for i in range(len(state)):
        if state[i] > 1 or state[i] < 0:
            print "INDEX OVER: " + str(i) + " STATE " + str(state[i])
            for j in obs:
                print("TAG: " + str(j) + " VALUE: " + str(obs[j]))
            print(state)
            raise ValueError("You're not normalizing correctly! Some of your values are either above one or below zero.")

if __name__ == "__main__":
    main()