"""The state_base module"""

from predicate_base import *


class State(object):
    """Holds a believed state."""

    def __init__(self, evaluator=None, predicates=()):
        super(State, self).__init__()
        self.dict = {}
        self.evaluator = evaluator
        self.hash = 0
        for p in predicates:
            self.put(p)

    def put(self, pred_inst):
        if not (pred_inst.pred, pred_inst.instance) in self.dict:
            self.hash += hash((pred_inst.pred, pred_inst.instance))

        self.dict[(pred_inst.pred, pred_inst.instance)] = pred_inst

    def __getitem__(self, pred_inst):
        key = (pred_inst.pred, pred_inst.instance)
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

    def get_by_tuple(self, t):
        """Get an item by a tuple.

        :param t: the tuple
        :return: The predicate instance identified by the tuple
        """
        return self.dict[t]

    def get_all(self, pred, val):
        """Get all predicate instances of the given predicate with the given value.

        :param pred: a predicate
        :param val:  a value
        :return: a list of all matching predicate instances
        """
        out = []
        for k, v in self.dict.iteritems():
            if k[0] == pred and val == v.val:
                out.append(v)

        return out

    def force_get(self, pred_inst):
        """Like __getitem__ but tries to evaluate the predicate, if it's not in the state.

        :param pred_inst: a predicate instance
        :return:
        """
        out = self.__getitem__(pred_inst)
        if out == None:
            out = self.evaluator.evaluate(pred_inst)
            self.put(out)

        return out

    def difference(self, other):
        """Check all fields of self against all fields of other.

        This forces evaluation of unknown predicates of other.
        """
        out = State()

        for k in self.dict:
            if self.dict[k] != other.force_get(self.dict[k]):
                out.put(self.dict[k])

        return out

    def combinable(self, other):
        """Check whether self can be unified with other.

        This does not force evaluation. Non-shared fields are assumed to unify.
        """
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

    def unify(self, other):
        """Unify self with other.

        If two fields don't match, the one of self overrides the one of other.
        This does not force evaluation. Non-shared fields are assumed to unify.
        """
        out = State()
        out.dict = dict(other.dict)

        for k in self.dict:
            out.put(self.dict[k])

        return out
