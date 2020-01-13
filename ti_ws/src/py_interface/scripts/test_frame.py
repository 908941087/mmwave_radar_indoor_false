import unittest
from Frame import Point, Frame, FrameService, StabilityMap, Blocks

class PointTestCase(unittest.TestCase):
    def test_add_positive_points(self):
        a = Point(0, 0)
        b = Point(1, 2)
        self.assertEqual(a + b, b)

    def test_add_negative_points(self):
        a = Point(-2, -1)
        b = Point(-5, -6)
        self.assertEqual(a + b, Point(-7, -7))

    def test_add_float_points(self):
        a = Point(1.0, 2.0)
        b = Point(0, 0)
        self.assertEqual(a + b, a)

class BlocksTestCase(unittest.TestCase):
    def setUp(self):
        self.blocks = Blocks(20, 20)
        self.blocks.resolution = 5

    def test_translate_axis(self):
        point = Point(-30, 45)
        self.assertEqual(self.blocks.translate_axis(point), (20, 55))

    def test_get_block_of_point_inside_map(self):
        p1 = Point(-30, 45)  # translated to Point(20, 55)
        p2 = Point(-50, 100)  # translated to Point(0, 0)
        p3 = Point(24.6, 38.3)  # translated to Point(74.6, 61.7)
        self.assertEqual(self.blocks.get_block(p1), (4, 11))
        self.assertEqual(self.blocks.get_block(p2), (0, 0))
        self.assertEqual(self.blocks.get_block(p3), (14, 12))

    def test_get_block_of_point_outside_map(self):
        """
        Point outside map is not allowed, they should be filtered out by passthrough_filter.
        Thus point outside map should raise an exception.
        """
        points = [Point(-50.000001, 50), 
            Point(50.000001, 50), 
            Point(0, 100.000001), 
            Point(0, -0.000001)]
        for point in points:
            self.assertRaises(IndexError, self.blocks.get_block, point)

    def test_get_neighbors(self):
        points = [Point(0, 50),   # translated to Point(50, 50), corresponding block is (10, 10)
                Point(-49.99999, 0.00001),   # translated to Point(0.00001, 99.99999), corresponding block is (0, 19)
                Point(-49.99999, 99.99999),   # translated to Point(0.00001, 0.00001), corresponding block is (0, 0)
                Point(49.99999, 99.99999),   # translated to Point(99.99999, 0.00001), corresponding block is (19, 0)
                Point(49.99999, 0.00001)]  # translated to Point(99.99999, 99.99999), corresponding block is (19, 19)
        neighbors = [[[10, 10], [9, 10], [10, 9], [9, 9]], 
                [[0, 19], [0, 19], [0, 19], [0, 19]], 
                [[0, 0], [0, 0], [0, 0], [0, 0]], 
                [[19, 0], [19, 0], [19, 0], [19, 0]], 
                [[19, 19], [19, 19], [19, 19], [19, 19]]]
        for point, neighbor in zip(points, neighbors):
            self.assertEqual(self.blocks.get_neighbors(point), neighbor)
        

if __name__ == '__main__':
    unittest.main()