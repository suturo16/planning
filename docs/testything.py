import random

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

	print('s1 & s2 = ' + str(s1.combinable(s2)))
	print('s1 & s3 = ' + str(s1.combinable(s3)))
	print('s1 & s4 = ' + str(s1.combinable(s4)))

	print('s1 / s2 = ' + str(s1 / s2))
	print('s1 + s3 = ' + str(s1 + s3))


