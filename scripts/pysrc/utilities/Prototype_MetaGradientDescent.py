import numpy as np


class MetaGradientDescent():
    def __init__(self, _dimensions, _startingPrototypes, non_linearity='sigmoid'):
        # initialize the parameters of a neural network
        self.dimensions = _dimensions
        self.num_features = _startingPrototypes - 1  # this is to account for the bias unit
        self.non_linearity = non_linearity
        # init the weights
        self.prototypes = np.random.normal(0.0, 1.0, size=(_dimensions, self.num_features))
        self.g = np.zeros((self.dimensions, self.num_features))

        # dynamically set functions !!!
        if non_linearity == 'sigmoid':
            def apply_nonlinearity(self, phi):
                return 1.0 / (1 + np.exp(-phi))

            setattr(self.__class__, 'apply_nonlinearity', apply_nonlinearity)

            def get_nonlinearity_derivative(self, phi):
                return phi * (1 - phi)

            setattr(self.__class__, 'get_nonlinearity_derivative', get_nonlinearity_derivative)

        # dynamically set functions!!!
        elif self.non_linearity == 'tanh':
            def apply_nonlinearity(self, phi):
                return np.tanh(phi)

            setattr(self.__class__, 'apply_nonlinearity', apply_nonlinearity)

            def get_nonlinearity_derivative(self, phi):
                return 1.0 - (phi ** 2)

            setattr(self.__class__, 'get_nonlinearity_derivative', get_nonlinearity_derivative)

        else:
            raise Exception('There is no implementation for the given non_linearity function ' + str(non_linearity))

        return

    def get_features(self, obs):
        phi = np.dot(self.prototypes.T, obs)
        phi = self.apply_nonlinearity(phi)
        self.phi = phi
        return np.concatenate([phi, np.array([1])])  # add bias unit

    def update_prototypes(self, obs, alpha, delta, th, _lambda=.5):

        F = np.array([self.get_nonlinearity_derivative(self.phi)])
        obs = np.array([obs]).T
        th = th[:self.num_features]
        alpha = alpha[:self.num_features]
        prototypes_delta_backprop_no_w = np.dot(obs, F)
        prototypes_delta_backprop = np.dot(obs, th * F)
        prototypes_delta_idbd = np.multiply(self.g, self.phi.T)
        self.prototypes += alpha * delta * (
            (1.0 - _lambda) * prototypes_delta_idbd + _lambda * prototypes_delta_backprop)
        self.g += alpha * (-np.multiply(self.g, (((1.0 - _lambda) * self.phi) ** 2).T) \
                           + prototypes_delta_backprop_no_w * delta \
                           - _lambda * np.multiply(prototypes_delta_backprop_no_w, np.multiply(self.phi, th).T))
