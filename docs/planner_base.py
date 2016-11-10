from predicate_base import *
from operator_base import *
from state_base import *
from predicate_evaluator import * 

import random
try:
	from Queue import PriorityQueue  # ver. < 3.0
except ImportError:
	from queue import PriorityQueue


class Plan(object):
	def __init__(self, op, head=None, tail=None):
		super(Plan, self).__init__()
		self.head = head
		self.tail = tail
		self.op = op
		self.cost = op.cost
		if self.tail != None:
			self.cost += tail.cost

	def __str__(self):
		if self.tail != None:
			return str(self.op) + str(self.tail)
		else:
			return str(self.op)


class PlanningProblem(object):
	"""docstring for PlanningProblem"""
	def __init__(self, start, goal, planL, planR, sortedOps):
		super(PlanningProblem, self).__init__()
		self.start = start
		self.goal  = goal
		self.planL = planL
		self.planR = planR
		self.ops = sortedOps
		self.opStack = []
		self.cost = 0
		if planL != None:
			self.cost += planL.cost

		if planR != None:
			self.cost += planR.cost

	def __cmp__(self, other):
		if other == None:
			return 1

		return -cmp(self.cost, other.cost)

	def __str__(self):
		return ('--PLANNING-PROBLEM--\nSTART' + 
				str(self.start) + '\nGOAL' + str(self.goal) + 
				'\nOperators: ' + str(self.ops) + 
				'\nOp-Stack: ' + str(self.opStack))
		
def subsets(a):
	if a == []:
		return set()

	out = set()
	subs = subsets(a[1:])

	print('subsets of ' + str(a))
	elems = a[:1][0]
	if len(elems) > 0:
		for x in elems:
			print('donald')
			out.add(State(preds=[x]))
			for s in subs:
				print('hillary')
				out.add(s)
				out.add(State(preds=[x]).unify(s))
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
		print('generate permutations for: ' + str(op) + '\n' + str(diff))
		i = 0
		for x in op.b_inst:
			postCon = op.b_inst.getByTuple(x)
			opts[i] = diff.getAll(postCon.pred, postCon.val)
			i += 1

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

			sortedOps = map(lambda t: t[1], sorted(map(lambda t: (t[0].cost / t[1], t[0]), opCount.iteritems())))

			return PlanningProblem(start, goal, planL, planR, sortedOps)

		return None

	def testPlan(self, constraints, plan):
		currentState = self.start
		cost = 0
		while plan.tail != None:
			diff = plan.op.a_inst.difference(start)
			cost += op.cost
			if len(diff) == 0:
				currentState = op.a_inst.unify(currentState)
				plan = plan.tail
			else:
				planL = plan.head
				planL.cost = cost
				plan.head = None
				newProblem = self.buildProblem(currentState, 
							   op.a_inst.unify(currentState), 
							   planL, 
							   plan)
				return False, newProblem

		return True, None

	def solveProblem(self, problem):
		print('solving problem...')
		if len(problem.opStack) == 0 and len(problem.ops) > 0:
			op = problem.ops[0]
			problem.ops = problem.ops[1:] 
			diff = goal.difference(start)
			problem.opStack = self.generatePermutations(op, diff)

		if len(problem.opStack) > 0:
			print('DFGHJKKJHGFDFGHJKJHG')
			for x in problem.opStack:
				print(x)

			op = problem.opStack[0]
			problem.opStack = problem.opStack[1:]

			newDiff = op.a_inst.difference(problem.start)

			if len(newDiff) > 0:
				newProblem = self.buildProblem(problem.start, 
											   op.a_inst, 
											   problem.planL, 
											   Plan(op, tail=problem.planR))
				if newProblem != None:
					self.problemHeap.put(newProblem)
			else:
				plan = Plan(op, head=problem.planL, tail=problem.planR)
				while plan.head != None:
					plan = plan.head

				ok, newProblem = self.testPlan(None, plan)

				if ok:
					return plan
				else:
					self.problemHeap.put(newProblem)

		return None

	def initPlanner(self, start, goal):
		initialProblem = self.buildProblem(start, goal, None, None)
		print(initialProblem)
		self.start = start
		self.goal = goal
		if initialProblem != None:
			self.problemHeap.put(initialProblem)
			return True
		else:
			return False

	def getNextPlan(self):
		plan = None
		#while plan == None and not self.problemHeap.empty():
		nextProblem = self.problemHeap.get()
		print(nextProblem)
		plan = self.solveProblem(nextProblem)

		# if plan != None:
		# 	return plan, 'Planning successful'
		# else:
		# 	return None, 'Planning failed! Out of options!'


if __name__ == '__main__':
	ppool = PredicatePool()
	ppool.loadFromXML('Test_Predicates.xml')

	oppool = OperatorPool()
	oppool.loadFromXML('Test_Operators.xml', ppool)

	planner = Planner(ppool, oppool)

	pi_1f = PredicateInstance(ppool['onTop'], ('apple', 'table'), False)
	pi_1t = PredicateInstance(ppool['onTop'], ('apple', 'table'), True)
	pi_2f = PredicateInstance(ppool['grasped'], ('apple',), False)
	pi_3f = PredicateInstance(ppool['container'], ('table',), False)

	baseState1 = State(preds=[pi_1f, pi_2f, pi_3f])
	evaluator = PseudoEvaluator(baseState1)

	start = State(evaluator=evaluator)
	goal = State(preds=[pi_1t])

	planner.initPlanner(start, goal)
	for x in range(3):
		plan = planner.getNextPlan()

	print(plan)

