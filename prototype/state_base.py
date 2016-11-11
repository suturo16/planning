from predicate_base import *


class State(object):
    """docstring for State"""

    def __init__(self, evaluator=None, preds=[]):
        super(State, self).__init__()
        self.dict = {}
        self.evaluator = evaluator
        self.hash = 0
        for p in preds:
            self.put(p)

    def put(self, predInst):
        if not (predInst.pred, predInst.instance) in self.dict:
            self.hash += hash((predInst.pred, predInst.instance))

        self.dict[(predInst.pred, predInst.instance)] = predInst

    def __getitem__(self, pred):
        key = (pred.pred, pred.instance)
        if key in self.dict:
            return self.dict[key]

        return None

    def __len__(self):
        return len(self.dict)

    def __hash__(self):
        return self.hash

    def __iter__(self):
        return self.dict.__iter__()

    def __str__(self):
        out = '--STATE--'
        for k, v in self.dict.iteritems():
            out += '\n  |-' + str(v)

        return out

    def iteritems(self):
        return self.dict.iteritems()

    def remove(self, key):
        self.dict.pop((key.pred, key.instance), None)
        self.hash -= hash((key.pred, key.instance))

    def getByTuple(self, t):
        return self.dict[t]

    def getAll(self, pred, val):
        out = []
        for k, v in self.dict.iteritems():
            if k[0] == pred and val == v.val:
                out.append(v)

        return out

    def forceGet(self, pred):
        out = self.__getitem__(pred)
        if out == None:
            out = self.evaluator.evaluate(pred)
            self.put(out)

        return out

    """
    Checking all fields of self against all fields of other.
    Forces evaluation of unknown predicates of other
    """
    def difference(self, other):
        out = State()

        for k in self.dict:
            if self.dict[k] != other.forceGet(self.dict[k]):
                out.put(self.dict[k])

        return out

    """
    Checks whether self can be unified with other. Does not
    force evaluation, so unknown fields are assumed to match.
    """
    def combinable(self, other):
        it = self
        itO = other

        if len(other) < len(self):
            it = other
            itO = self

        for k in it.dict:
            if k in itO:
                if it.dict[k].val != itO.dict[k].val:
                    return False

        return True

    """
    Unifies self with other. If two fields don't match, the one of self overrides the one of other.
    This does not force evaluation. Non-shared fields are assumed to unify.
    """
    def unify(self, other):
        out = State()
        out.dict = dict(other.dict)

        for k in self.dict:
            out.put(self.dict[k])

        return out
