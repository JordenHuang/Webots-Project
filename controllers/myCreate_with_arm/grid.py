
class Grid:
    GRID_UNKNOWN = -1
    GRID_OK = 0
    GRID_HAS_OBSTACLE = 1

    FACE_UP = 0
    FACE_RIGHT = 1
    FACE_DOWN = 2
    FACE_LEFT = 3
    # Order matters
    # FACES = [
    #     FACE_UP,
    #     FACE_RIGHT,
    #     FACE_DOWN,
    #     FACE_LEFT,
    # ]

    def __init__(self, cellSize):
        # meter
        self.cellSize = cellSize
        self.grid = [[self.GRID_OK]]
        self.x = 0
        self.y = 0
        self.originX = 0
        self.originY = 0
        self.direction = self.FACE_LEFT

    def getFront(self):
        # FIXME: Fix this, not the current grid, but the front one
        return self.grid[self.y][self.x]

    def turnFace(self, to:str):
        if to == "back":
            self.direction = (self.direction + 2) % 4
        elif to == "left":
            self.direction = (self.direction + 3) % 4
        elif to == "right":
            self.direction = (self.direction + 1) % 4

    def setGrid(self, status):
        # up
        if self.direction == self.FACE_UP:
            self.y -= 1
            while self.y < 0:
                self.grid.insert(0, [self.GRID_UNKNOWN] * len(self.grid[0]))
                self.y += 1
                self.originY += 1

        # down
        elif self.direction == self.FACE_DOWN:
            self.y += 1
            while self.y >= len(self.grid):
                self.grid.append([self.GRID_UNKNOWN] * len(self.grid[0]))

        # right
        elif self.direction == self.FACE_RIGHT:
            self.x += 1
            while self.x >= len(self.grid[0]):
                for row in self.grid:
                    row.append(self.GRID_UNKNOWN)

        # left
        elif self.direction == self.FACE_LEFT:
            self.x -= 1
            while self.x < 0:
                for row in self.grid:
                    row.insert(0, self.GRID_UNKNOWN)
                self.x += 1
                self.originX += 1

        self.grid[self.y][self.x] = status

    def moveForward(self):
        self.setGrid(self.GRID_OK)

    def moveBackward(self):
        self.turnFace("back")
        self.setGrid(self.GRID_OK)
        self.turnFace("back")

    def turnLeft(self):
        self.setGrid(self.GRID_HAS_OBSTACLE)
        self.turnFace("back")
        self.setGrid(self.GRID_OK)
        self.turnFace("back")

        self.turnFace("left")

    def turnRight(self):
        self.setGrid(self.GRID_HAS_OBSTACLE)
        self.turnFace("back")
        self.setGrid(self.GRID_OK)
        self.turnFace("back")

        self.turnFace("right")

    def displayGrid(self):
        for r in range(len(self.grid)):
            print(f"{r:2d}: ", end='')
            for c in range(len(self.grid[0])):
                # current position
                if r == self.y and c == self.x:
                    print("@", end='')
                # original point
                elif r == self.originY and c == self.originX:
                    print("*", end='')
                # ok
                elif self.grid[r][c] == self.GRID_OK:
                    print("o", end='')
                # obstacle
                elif self.grid[r][c] == self.GRID_HAS_OBSTACLE:
                    print("x", end='')
                # unknown
                else:
                    print("_", end='')
            print()
        
        print(f"face: {self.direction}")

