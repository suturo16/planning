"""The preodicate_base module.


"""

import xml.etree.ElementTree as ET
from state_base import *


class Predicate(object):
    """Represents a predicate.
    
    This class overrides builtin methods for usage on predicates.
    """

    def __init__(self, name, arguments):
        super(Predicate, self).__init__()
        self.name = name
        self.fields = arguments

    def __str__(self):
        return self.name + str(self.fields)

    def __eq__(self, other):
        return self.name == other.name

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.name.__hash__()

    def __len__(self):
        return len(self.fields)

    def __getitem__(self, idx):
        return self.fields[idx]


class PredicateInstance(object):
    """Represents an instance of a predicate and it's value.

    This class overrides builtin methods for usage on predicate instances.
    """

    def __init__(self, pred, instance, val):
        super(PredicateInstance, self).__init__()
        self.pred = pred
        self.instance = instance
        self.val = val
        assert(len(self.instance) == len(self.pred))

    def __eq__(self, other):
        return other is not None and self.pred == other.pred and self.instance == other.instance and self.val == other.val

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.pred.__hash__() + self.instance.__hash__()

    def __bool__(self):
        return self.val

    def __nonzero__(self):
        return self.__bool__()

    def __str__(self):
        return self.pred.name + str(self.instance) + ' := ' + str(self.val)

    def __getitem__(self, idx):
        return self.instance[idx]

    def __len__(self):
        return len(self.instance)

    def __iter__(self):
        return self.instance.__iter__()


class PredStruct(object):
    """Holds two states."""
    def __init__(self, a_state, b_state):
        super(PredStruct, self).__init__()
        self.a_state = a_state
        self.b_state = b_state

    def __str__(self):
        return 'Side-A:' + str(self.a_state) + '\nSide-B:' + str(self.b_state)

    def __hash__(self):
        return self.a_state.__hash__() + self.b_state.__hash__()

    def __eq__(self, other):
        return other is not None and self.a_state == other.a_state and self.b_state == other.b_state

    def __ne__(self, other):
        return not self.__eq__(other)


class ParameterizedPredStruct(PredStruct):
    """Holds two predicate instances and parameterizes them."""
    
    def __init__(self, params, a_state, b_state):
        super(ParameterizedPredStruct, self).__init__(a_state, b_state)
        self.params = params

    def __str__(self):
        return '--PARAMETERIZED--\n' + super(ParameterizedPredStruct, self).__str__()

    def parameterize(self, param, inst):
        """
        Parameterize a state.

        Details.
        :param param: state
        :param inst:
        :return:
        """
        resolved = {}

        p_inst = State(evaluator=param.evaluator)
        p_inst.dict = dict(param.dict)

        for l, i in inst.iteritems():
            for n, a in p_inst.iteritems():
                if a.pred == i.pred and a.val == i.val:
                    for x in range(len(a.pred.fields)):
                        if not a[x] in resolved:
                            resolved[a[x]] = i[x]

                    p_inst.remove(a)
                    break

        return resolved

    def buildSolution(self, resolved):
        # print(resolved)

        if len(resolved) == len(self.params):
            finalA = State()
            finalB = State()

            for v, a in self.a_state.iteritems():
                t = []
                for x in a:
                    t.append(resolved[x])

                finalA.put(PredicateInstance(a.pred, tuple(t), a.val))

            for v, b in self.b_state.iteritems():
                t = []
                for x in b:
                    t.append(resolved[x])

                finalB.put(PredicateInstance(b.pred, tuple(t), b.val))

            return PredStruct(finalA, finalB)

        return None

    def parameterizeA(self, inst):
        return self.buildSolution(self.parameterize(self.a_state, inst))

    def parameterizeB(self, inst):
        return self.buildSolution(self.parameterize(self.b_state, inst))


class PredicatePool(object):
    """Loads and holds predicates from a xml file."""

    def __init__(self):
        super(PredicatePool, self).__init__()
        self.predicates = {}

    def loadFromXML(self, file_path):
        """
        Loads predicates from a xml file into self.predicates.

        :param file_path: Path to the xml file.
        :return: None
        """
        tree = ET.parse(file_path)
        root = tree.getroot()

        for p in root:
            name = p.attrib['name']
            self.predicates[name] = Predicate(name, tuple(p.attrib['args'].split()))

    def __getitem__(self, key):
        return self.predicates[key]

    def __contains__(self, key):
        return key in self.predicates


if __name__ == '__main__':
    pool = PredicatePool()
    pool.loadFromXML('Test_Predicates.xml')

    p1 = pool['onTop']  # Predicate('onTop', ('a', 'b'))
    p2 = pool['rightOf']  # Predicate('rightOf', ('a', 'b'))

    p3 = pool['grasped']  # Predicate('inHand', ('a',))
    p4 = pool['inFront']  # Predicate('underneath', ('a', 'b'))
    p5 = pool['inReach']  # Predicate('reachable', ('a',))

    print(p1)
    print(p2)
    print(p3)
    print(p4)

    print('onTop == rightOf ' + str(p1 == p2))

    pi1 = PredicateInstance(p1, ('apple', 'desk'), True)
    pi2 = PredicateInstance(p2, ('apple', 'desk'), False)
    pi3 = PredicateInstance(p1, ('orange', 'desk'), True)

    postcon1 = PredicateInstance(p1, ('a', 'b'), True)
    postcon2 = PredicateInstance(p1, ('c', 'b'), True)
    postcon3 = PredicateInstance(p3, ('a',), False)
    postcon4 = PredicateInstance(p3, ('c',), False)

    precon1 = PredicateInstance(p3, ('a',), True)
    precon2 = PredicateInstance(p3, ('c',), True)
    precon3 = PredicateInstance(p5, ('b',), True)

    print(pi1)
    print(pi2)

    w = ParameterizedPredStruct(['a', 'b', 'c'],
                                State(predicates=[precon1, precon2, precon3]),
                                State(predicates=[postcon1, postcon2, postcon3, postcon4]))
    print(w)

    print("-"*10)
    resolve = w.parameterizeB(State(predicates=[pi1, pi3]))
    print(resolve)
