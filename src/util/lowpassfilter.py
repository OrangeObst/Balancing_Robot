class LowPassFilter:
        def __init__(self, alpha):
            self.alpha = alpha
            self.y = 0.0
        
        def filter(self, x):
            self.y = self.alpha * x + (1 - self.alpha) * self.y
            return self.y