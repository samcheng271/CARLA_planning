import unittest
from unittest.mock import MagicMock
import carla
from pqd import D_star
from queue import PriorityQueue

class TestInit(unittest.TestCase):
    def setUp(self):
        self.waypoint = MagicMock()
        self.start_location = MagicMock()
        self.goal_location = MagicMock()
        self.vehicle = MagicMock()
        self.world = MagicMock()
        self.map = MagicMock()

        self.start_location.transform = MagicMock()
        self.goal_location.transform = MagicMock()
        self.start_location.transform.location = MagicMock()
        self.goal_location.transform.location = MagicMock()

        self.vehicle.get_location.return_value = self.start_location
        self.map.get_waypoint.side_effect = [MagicMock(name='start_wp_mock'), MagicMock(name='goal_wp_mock')]
        self.map.generate_waypoints.return_value = [MagicMock()] * 10


    def test_dstar_init(self):
        d_star = D_star(
            waypoint=self.waypoint,
            start_location=self.start_location,
            goal_location=self.goal_location,
            vehicle=self.vehicle,
            world=self.world,
            map=self.map,
            resolution=2.0
        )
        self.map.get_waypoint.assert_any_call(self.start_location.transform.location)
        self.map.get_waypoint.assert_any_call(self.goal_location.transform.location)

        self.assertEqual(d_star.settings, 'CollisionChecking')
        self.assertEqual(d_star.resolution, 2.0)
        self.assertEqual(d_star.waypoint, self.waypoint)
        self.assertEqual(d_star.obstacle_threshold, 3.0)
        self.assertIsInstance(d_star.b, dict)
        self.assertIsInstance(d_star.OPEN, PriorityQueue)
        self.assertEqual(d_star.tag, {})
        self.assertEqual(d_star.V, set())
        self.assertEqual(d_star.parameters, ())
        self.assertEqual(d_star.ind, 0)
        self.assertEqual(d_star.Path, [])
        self.assertFalse(d_star.done)
        self.assertEqual(d_star.Obstaclemap, {})
        self.assertEqual(d_star.world, self.world)
        self.assertEqual(d_star.map, self.map)
        self.assertEqual(d_star.vehicle, self.vehicle)
        self.assertEqual(d_star.state, self.waypoint)
        self.assertEqual(d_star.location, self.vehicle.get_location())
        self.assertEqual(d_star.state_space, self.map.get_waypoint(self.vehicle.get_location(), project_to_road=True))
        self.assertEqual(len(d_star.waypoints), 10)
        self.assertEqual(d_star.h, {})
        self.assertEqual(d_star.start_location, self.start_location)
        self.assertEqual(d_star.goal_location, self.goal_location)
        self.assertEqual(d_star.x0, self.map.get_waypoint(self.start_location.transform.location))
        self.assertEqual(d_star.xt, self.map.get_waypoint(self.goal_location.transform.location))

    def test_populate_open(self):
        d_star = D_star(
            waypoint=self.waypoint,
            start_location=self.start_location,
            goal_location=self.goal_location,
            vehicle=self.vehicle,
            world=self.world,
            map=self.map,
            resolution=2.0
        )

        next_waypoint = MagicMock()
        d_star.state_space.next.return_value = [next_waypoint]

        d_star.cost = MagicMock(return_value=10.0)
        open_queue = d_star.populate_open()
        d_star.cost.assert_called_with(d_star.state_space, next_waypoint)
        self.assertIsInstance(open_queue, PriorityQueue)
        self.assertFalse(open_queue.empty())

        get_item = open_queue.get()
        self.assertIsInstance(get_item, tuple)
        self.assertEqual(len(get_item), 2)

        key, state_space = get_item

        self.assertIsInstance(key, float)
        self.assertEqual(state_space, d_star.state_space)

    def test_cost(self):
        d_star = D_star(
            waypoint=self.waypoint,
            start_location=self.start_location,
            goal_location=self.goal_location,
            vehicle=self.vehicle,
            world=self.world,
            map=self.map,
            resolution=2.0
        )

        start = MagicMock()
        start.transform.location = carla.Location(x=0, y=0, z=0)
        goal = MagicMock()
        goal.transform.location = carla.Location(x=3, y=4, z=0)

        cost = d_star.cost(start, goal)
        self.assertAlmostEqual(cost, 5.0, places=2)
        
if __name__ == '__main__':
    unittest.main()
