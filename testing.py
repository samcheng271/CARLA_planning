import unittest
import carla
import random
from pqd import D_star

class TestDStar(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        cls.client = carla.Client('localhost', 2000)
        cls.client.set_timeout(10.0)
        cls.world = cls.client.get_world()
        cls.map = cls.world.get_map()

        blueprint_library = cls.world.get_blueprint_library()
        firetruck_bp = blueprint_library.filter('vehicle.carlamotors.firetruck')[0]
        spawn_points = cls.map.get_spawn_points()
        point_a = spawn_points[0]
        point_b = random.choice(spawn_points)
        while point_b.location == point_a.location:
            point_b = random.choice(spawn_points)

        cls.start_waypoint = cls.map.get_waypoint(point_a.location)
        cls.end_waypoint = cls.map.get_waypoint(point_b.location)
        cls.firetruck = cls.world.spawn_actor(firetruck_bp, point_a)
        
    @classmethod
    def tearDownClass(cls):
        cls.firetruck.destroy()

    def test_initialization(self):
        D = D_star(1, self.start_waypoint, self.end_waypoint, self.world, map)

        
        self.assertIsInstance(D.waypoints, list)
        self.assertIsInstance(D.x0, carla.Waypoint)
        self.assertIsInstance(D.xt, carla.Waypoint)
        self.assertEqual(len(D.waypoints), len(self.map.generate_waypoints(D.resolution)))
        self.assertEqual(D.x0.transform.location, self.start_waypoint.transform.location)
        self.assertEqual(D.xt.transform.location, self.end_waypoint.transform.location)

    def test_populate_open(self):
        D = D_star(1, self.start_waypoint, self.end_waypoint, self.firetruck, self.world, self.map)
        D.populate_open()
        
        self.assertFalse(D.OPEN.empty())
        item = D.OPEN.get()
        print(f"Open: {item}")
        self.assertIsInstance(item, tuple)
        self.assertIsInstance(item[0], float)
        self.assertTrue(item[0] <= 10)
        self.assertIsInstance(item[1], carla.Waypoint)
        print(f"Populate Open Test Passed: OPEN Queue Item - {item}")
    
    """
    def test_checkState_waypoint(self):
        # Test checkState with a carla.Waypoint object
        D = D_star(1, self.start_waypoint, self.end_waypoint, self.firetruck, self.world, self.map)
        waypoint = D.waypoints[0]

        D.checkState(waypoint)

        waypoint_id = waypoint.id
        self.assertIn(waypoint_id, D.h)
        self.assertEqual(D.h[waypoint_id], 0)
        self.assertIn(waypoint_id, D.tag)
        self.assertEqual(D.tag[waypoint_id], 'New')
        print(f"Test Passed for Waypoint: h[{waypoint_id}] = {D.h[waypoint_id]}, tag[{waypoint_id}] = {D.tag[waypoint_id]}")

    def test_checkState_non_waypoint(self):
        # Test checkState with a non-Waypoint ID
        D = D_star(1, self.start_waypoint, self.end_waypoint, self.firetruck, self.world, self.map)
        non_waypoint_id = 999  # Assuming this is not an actual Waypoint ID

        D.checkState(non_waypoint_id)

        self.assertIn(non_waypoint_id, D.h)
        self.assertEqual(D.h[non_waypoint_id], 0)
        self.assertIn(non_waypoint_id, D.tag)
        self.assertEqual(D.tag[non_waypoint_id], 'New')
        print(f"Test Passed for Non-Waypoint ID: h[{non_waypoint_id}] = {D.h[non_waypoint_id]}, tag[{non_waypoint_id}] = {D.tag[non_waypoint_id]}")
    """
if __name__ == '__main__':
    unittest.main()