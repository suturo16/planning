class ArgumentBase(object):
    """docstring for ArgumentBase"""
    def __init__(self, value):
        super(ArgumentBase, self).__init__()
        self.isPlaceholder = True
        self.value = value

    def __str__(self):
        return '('+str(self.value)+')'
        
    def __hash__(self):
        return hash(self.value) + hash(self.isPlaceholder)

    def __eq__(self, other):
        return other != None and self.value == other.value and self.isPlaceholder == other.isPlaceholder

    def __ne__(self, other):
        return not self.__eq__(other)

class Argument(ArgumentBase):
    """docstring for Argument"""
    def __init__(self, value):
        super(Argument, self).__init__(value)
        self.isPlaceholder = False
        
    def __str__(self):
        return str(self.value)