class NavigationPath:
    def __init__(self, instructions, miles, kilometers, time):
        self.instructions = instructions
        self.miles = miles
        self.kilometers = kilometers
        self.time = time

    def GetInstructions(self):
        return self.instructions
    def GetPathLength(self):
        return (self.miles,self.kilometers, self.time)
