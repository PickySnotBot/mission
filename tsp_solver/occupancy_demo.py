class MockOccupancyGrid:
    def __init__(self, width, height, resolution, origin_x, origin_y, data=None):
        self.info = self.Info(width, height, resolution, origin_x, origin_y)
        if data is None:
            # Create an empty grid (all cells unoccupied)
            self.data = [0] * (width * height)
        else:
            self.data = data

    class Info:
        def __init__(self, width, height, resolution, origin_x, origin_y):
            self.width = width
            self.height = height
            self.resolution = resolution
            self.origin = self.Origin(origin_x, origin_y)

        class Origin:
            def __init__(self, x, y):
                self.position = self.Position(x, y)

            class Position:
                def __init__(self, x, y):
                    self.x = x
                    self.y = y
