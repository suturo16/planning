from __future__ import division

import random
try:
    from Queue import PriorityQueue  # ver. < 3.0
except ImportError:
    from queue import PriorityQueue

class Predicate(object):
	"""docstring for Predicate"""
	def __init__(self, name, fields):
		super(Predicate, self).__init__()
		self.name = name
		self.fields = fields
		
	def __str__(self):
		return self.name + str(self.fields)

	def __hash__(self):
		return str(self).__hash__()

	def __eq__(self, other):
		return str(self) == str(other)


class State(object):
	"""docstring for State"""
	def __init__(self):
		self.predicates = {}

	def evaluatePredicate(self, pred):
		return bool(random.getrandbits(1))

	def __getitem__(self, pred):
		if pred in self.predicates:
			return self.predicates[pred]
		else:
			val = self.evaluatePredicate(pred)
			self.predicates[pred] = val
			return val 
		
	def __len__(self):
		return len(self.predicates)

	def __contains__(self, pred):
		return pred in self.predicates

	def __setitem__(self, pred, val):
		self.predicates[pred] = val

	def __str__(self):
		out = 'state:'
		for k in self.predicates:
			out += '\n    ' + k + ' = ' + str(self.predicates[k])

		return out

	def __iter__(self):
		return dict.__iter__(self.predicates)

	def __add__(self, other):
		out = State()

		for k in other:
			out[k] = other[k]

		for k in self:
			if not k in out:
				out[k] = self[k]
			elif out[k] != self[k]:
				return None

		return out


	def __truediv__(self, other):
		"""Returns the values of b conflicting with a"""
		out = State()

		for k in other:
			if self[k] != other[k]:
				out[k] = other[k] 

		return out

	def length(self):
		return self.predicates

	def combinable(self, other):
		if len(other) >= len(self):
			for k in other:
				if k in self and self[k] != other[k]:
					return False
		else:
			for k in self:
				if k in other and self[k] != other[k]:
					return False

		return True


class Operator(object):
	"""docstring for Operator"""
	def __init__(self, preconditions, postconditions, cost):
		super(Operator, self).__init__()
		self.preconditions = preconditions
		self.postconditions = postconditions
		self.cost = cost

	def __lt__(self, other):
		return self.cost < other.cost

	def __le__(self, other):
		return self.cost <= other.cost

	def __gt__(self, other):
		return self.cost > other.cost

	def __ge__(self, other):
		return self.cost >= other.cost
		



class Path(list):
	"""docstring for Path"""
	def __init__(self):
		super(Path, self).__init__()
		self.cost = 0
		


class PlanningProblem(object):
	"""docstring for PlanningProblem"""
	def __init__(self, start, goal, left, right, ops):
		super(PlanningProblem, self).__init__()
		self.start = start
		self.goal = goal
		self.left = left
		self.right = right
		self.ops = ops
		
	def cost():
		return left.cost + right.cost

	def __lt__(self, other):
		return self.cost() < other.cost()

	def __le__(self, other):
		return self.cost() <= other.cost()

	def __gt__(self, other):
		return self.cost() > other.cost()

	def __ge__(self, other):
		return self.cost() >= other.cost()	


class Planner(object):
	"""docstring for Planner"""
	def __init__(self, operators):
		super(Planner, self).__init__()
		self.operators = operators
		

	def testPath(self, start, goal, constraints, path):
		currentState = start
		lastIndex = 0
		for x in range(len(path)):
			lastIndex = x
			op = path[x]
			# Check whether there are conflicts between pre conditions and state
			if len(currentState / op.preconditions) == 0:
				for k in op.postconditions:
					currentState[k] = op.postconditions[k]
			else:
				break

		lastIndex += 1
		diff = currentState / goal
		if len(diff) == 0:
			return True, currentState, path, [] 
		else:
			return False, currentState, path[:lastIndex], path[lastIndex:]

	def beginPlanning(self, currentState, goalState):
		self.currentState = currentState
		self.goalState = goalState
		self.problems = PriorityQueue()

		diff = currentState / goalState

		if len(diff) > 0:


			bp = PlanningProblem(self.currentState, goalState, Path(), Path())



	def getPlan(self):
		pass


if __name__ == '__main__':

	s1 = State()

	s1['f(x)'] = True
	s1['g(a, b)'] = False
	s1['h(c, v)'] = True

	s2 = State()
	s2['f(x)'] = False
	s2['g(a, b)'] = False
	s2['h(c, v)'] = True

	s3 = State()
	s3['z(x)'] = True

	s4 = State()
	s4['z(x)'] = True
	s4['f(x)'] = True	

	print(str(s1))
	print(str(s2))
	print(str(s3))
	print(str(s4))

	print('s1 & s2 = ' + str(s1.combinable(s2)))
	print('s1 & s3 = ' + str(s1.combinable(s3)))
	print('s1 & s4 = ' + str(s1.combinable(s4)))

	print('s1 / s2 = ' + str(s1 / s2))
	print('s1 + s3 = ' + str(s1 + s3))


