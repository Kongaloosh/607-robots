__author__ = 'kongaloosh'


def constant(gamma, data=None):
    return gamma


def increasing(gamma, data=None):
    if gamma == 1:
        return 0
    else:
        return gamma + 1./5.


def end_when_stationary_2(gamma, data):
    if data[3] is False:
        return 1
    else:
        return gamma


def end_when_stationary_3(gamma, data):
    if data[10] is False:
        return 1
    else:
        return gamma

