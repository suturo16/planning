from predicate_base import *
import xml.etree.ElementTree as ET

class OpRelations(object):
	"""docstring for OpRelations"""
	def __lt__(self, other):
		return self.cost < other.cost

	def __le__(self, other):
		return self.cost <= other.cost

	def __gt__(self, other):
		return self.cost > other.cost

	def __ge__(self, other):
		return self.cost >= other.cost

	def __cmp__(self, other):
		return cmp(self.cost, other.cost)

		

class Operator(PredStruct, OpRelations):
	"""docstring for Operator"""
	def __init__(self, name, preCon, postCon, cost):
		super(Operator, self).__init__(preCon, postCon)
		self.name = name
		self.cost = cost

	def __hash__(self):
		return self.name.__hash__() + self.cost.__hash__() + super(Operator, self).__hash__()

	def __eq__(self, other):
		return self.name == other.name and self.cost == other.cost and super(Operator, self).__hash__(other)

	def __str__(self):
		return 'OPERATOR-Instance: "' + self.name + '" cost:= ' + str(self.cost) + '\n' + super(Operator, self).__str__()


class OperatorTemplate(ParameterizedPredStruct, OpRelations):
	"""docstring for Operator"""
	def __init__(self, name, params, preCon, postCon, cost):
		super(OperatorTemplate, self).__init__(params, preCon, postCon)
		self.name = name
		self.cost = cost

	def __str__(self):
		return 'OPERATOR-Template: "' + self.name + '" cost:= ' + str(self.cost) + '\n' + super(OperatorTemplate, self).__str__()

"""
Idea: store the operators as a map |predicates -> bool -> operator| this should make it easier to find
	  suiting operators. 
"""
class OperatorPool(object):
	def __init__(self):
		super(OperatorPool, self).__init__()
		self.ops = {}

	def loadFromXML(self, filepath, predPool):
		tree = ET.parse(filepath)
		root = tree.getroot()

		for o in root:
			preCon = State()
			postCon = State()
			params = []

			operator = OperatorTemplate(o.attrib['name'], params, preCon, postCon, float(o.attrib['cost']))

			for c in o:
				if c.tag == 'preconditions':
					for p in c:
						name = p.attrib['name']
						if name in predPool:
							pp = tuple(p.attrib['args'].split())
							val = p.attrib['value'] in ['1', 'true']
							preCon.put(PredicateInstance(predPool[name], pp, val))

							for x in pp:
								if x not in params:
									params.append(x)

				elif c.tag == 'postconditions':
					for p in c:
						name = p.attrib['name']
						if name in predPool:
							pp = tuple(p.attrib['args'].split())
							val = p.attrib['value'] in ['1', 'true']
							pred = predPool[name]
							postCon.put(PredicateInstance(pred, pp, val))

							if not pred in self.ops:
								self.ops[pred] = {True : [], False : []}

							self.ops[pred][val].append(operator)

							for x in pp:
								if x not in params:
									params.append(x)
		

	def __str__(self):
		out = 'Operator-Pool'

		for p in self.ops:
			out += '  \n' + str(p)
			for x in self.ops[p]:
				out += '\n    ' + str(x)
				for o in self.ops[p][x]:
					out += '\n      ' + o.name + '(' + str(o.cost) + ')' 

		return out

	def getOperators(self, pred, val):
		if pred in self.ops:
			return list(self.ops[pred][val]) 

		return []

if __name__ == '__main__':
	ppool = PredicatePool()
	ppool.loadFromXML('Test_Predicates.xml')


	oppool = OperatorPool()
	oppool.loadFromXML('Test_Operators.xml', ppool)

	print(oppool)


