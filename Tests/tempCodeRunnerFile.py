class tracker:
    def __init__(self, filename):
        self.file = open(filename, 'r')
        self.reader = csv.reader(self.file)

    def track(self):
        try:
            next_line = next(self.reader)
            x, y = map(int, next_line)  
            return x, y
        except StopIteration:
            return None

    def close(self):
        self.file.close()