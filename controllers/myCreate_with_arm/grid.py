
class Grid:
    GRID_UNKNOWN = -1
    GRID_OK = 0
    GRID_HAS_OBSTACLE = 1

    FACE_UP = 0
    FACE_RIGHT = 1
    FACE_DOWN = 2
    FACE_LEFT = 3

    DIR_OFFSETS = {
        FACE_UP: (0, -1),
        FACE_RIGHT: (1, 0),
        FACE_DOWN: (0, 1),
        FACE_LEFT: (-1, 0),
    }

    def __init__(self, cellSize):
        self.cellSize = cellSize
        self.grid = [[self.GRID_OK]]
        self.originX = 0
        self.originY = 0
        self.x = 0
        self.y = 0
        self.prevX = 0
        self.prevY = 0
        self.direction = self.FACE_LEFT

    def restorePrev(self):
        self.x = self.prevX
        self.y = self.prevY

    def getOffsetCoord(self, direction):
        dx, dy = self.DIR_OFFSETS[direction]
        return self.x + dx, self.y + dy

    def expandToInclude(self, x, y):
        while y < 0:
            self.grid.insert(0, [self.GRID_UNKNOWN] * len(self.grid[0]))
            y += 1
            self.originY += 1
            self.y += 1
        while y >= len(self.grid):
            self.grid.append([self.GRID_UNKNOWN] * len(self.grid[0]))

        while x < 0:
            for row in self.grid:
                row.insert(0, self.GRID_UNKNOWN)
            x += 1
            self.originX += 1
            self.x += 1
        while x >= len(self.grid[0]):
            for row in self.grid:
                row.append(self.GRID_UNKNOWN)

    def mark(self, status):
        self.grid[self.y][self.x] = status

    def markPrevious(self, status):
        self.grid[self.prevY][self.prevX] = status

    def markFront(self, status):
        fx, fy = self.getOffsetCoord(self.direction)
        self.expandToInclude(fx, fy)
        fx, fy = self.getOffsetCoord(self.direction)
        self.grid[fy][fx] = status

    def turnFace(self, to):
        if to == "back":
            self.direction = (self.direction + 2) % 4
        elif to == "left":
            self.direction = (self.direction + 3) % 4
        elif to == "right":
            self.direction = (self.direction + 1) % 4

    def move(self):
        self.prevX, self.prevY = self.x, self.y
        self.x, self.y = self.getOffsetCoord(self.direction)
        self.expandToInclude(self.x, self.y)

    def moveForward(self):
        self.move()
        self.mark(self.GRID_OK)

    def moveBackward(self):
        self.turnFace("back")
        self.move()
        self.mark(self.GRID_OK)
        self.turnFace("back")

    def turnLeft(self):
        self.turnFace("left")
        # self.move()
        self.mark(self.GRID_OK)

    def turnRight(self):
        self.turnFace("right")
        # self.move()
        self.mark(self.GRID_OK)

    def displayGrid(self):
        for r, row in enumerate(self.grid):
            print(f"{r:2d}: ", end='')
            for c, cell in enumerate(row):
                if r == self.y and c == self.x:
                    print("@", end='')
                elif r == self.originY and c == self.originX:
                    print("*", end='')
                elif cell == self.GRID_OK:
                    print("o", end='')
                elif cell == self.GRID_HAS_OBSTACLE:
                    print("x", end='')
                else:
                    print("_", end='')
            print()
        print(f"face: {self.direction}")
