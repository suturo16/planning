from predicate_base import *


class PseudoEvaluator(object):
    """docstring for PseudoEvaluator"""

    def __init__(self, state):
        super(PseudoEvaluator, self).__init__()
        self.state = state

    def evaluate(self, pred):
        return self.state[pred]
