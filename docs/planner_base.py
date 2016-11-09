from predicate_base import *
from operator_base import *
from state_base import *

import random
try:
	from Queue import PriorityQueue  # ver. < 3.0
except ImportError:
	from queue import PriorityQueue


class PlanningProblem(object):
	"""docstring for PlanningProblem"""
	def __init__(self, planL, planR, sortedOps):
		super(PlanningProblem, self).__init__()
		self.planL = planL
		self.planR = planR
		self.ops = sortedOps
		self.opStack = []
		self.cost = planR.cost + planL.cost
		
def subsets(a):
	if a == []:
		return set()

	out = set()
	subs = subsets(a[1:])

	elems = a[:1][0]
	if len(elems) > 0:
		for x in elems:
			out.add((x,))
			for s in subs:
				out.add(s)
				out.add((x,) + s)
	else:
		return subs

	return out


class Planner(object):
	"""docstring for Planner"""
	def __init__(self, predBase, opBase):
		super(Planner, self).__init__()
		self.predBase = predBase
		self.opBase = opBase
		self.problemHeap = PriorityQueue()

	def generatePermutations(self, op, diff):
		opts = [None] * len(op.b_inst)
		for x in range(len(op.b_inst)):
			postCon = op.b_inst[x] 
			opts[x] = diff.getAll(postCon.pred, postCon.val)

		out = set()
		subs = subsets(opts)
		for x in subs:
			ps = op.parameterizeB(x)
			if ps != None:
				out.add(Operator(op.name, ps.a_inst, ps.b_inst, op.cost))

		return list(out)


	def buildProblem(self, start, goal, planL, planR):
		diff = goal.difference(start)

		if len(diff) > 0:
			types = set()
			opCount = {}

			for d in diff:
				predinst = diff.dict[d]
				key = (predinst.pred, predinst.val)
				if not key in types:
					types.add(key)
					ops = self.opBase.getOperators(key[0], key[1])

					if len(ops) == 0:
						print('Unsolvable problem: "' + str(predinst) + '" can not be achieved!')
						return None

					for o in ops:
						if o in opCount:
							opCount[o] = opCount[o] + 1
						else:
							opCount[o] = 1

			sortedOps = map(lambda t: t[1], sorted(map(lambda t: (t[1] * t[0].cost, t[1]), opCount.iteritems())))

			return PlanningProblem(planL, planR, sortedOps)

		return None


	def solveProblem(self, problem):
		if len(problem.opStack) == 0 and len(problem.ops) > 0:
			op = problem.ops[0]
			problem.ops = problem.ops[1:] 
			diff = goal.difference(start)
			problem.opStack = self.generatePermutations(op, diff)

		if len(problem.opStack) > 0:
			op = problem.opStack[0]
			problem.opStack = problem.opStack[1:]

			newDiff = op.a_inst.difference(problem.start)

			if len(newDiff) > 0:
				newProblem = self.buildProblem(problem.start. )
			else:









