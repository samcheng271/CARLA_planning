import unittest
from unittest.mock import MagicMock, patch
from queue import PriorityQueue
import carla  
from pqd import D_star

class TestDStarPopulateOpen(unittest.TestCase):
    def setUp(self):
    # Set up the mock environment
        self.mock_waypoint = MagicMock(spec=carla.Waypoint)
        self.mock_state_space = MagicMock(spec=carla.Waypoint)
        self.mock_vehicle = MagicMock(spec=carla.Vehicle)
        self.mock_world = MagicMock(spec=carla.World)
        self.mock_map = MagicMock(spec=carla.Map)

    # Mock the state space
        self.mock_vehicle.get_location.return_value = MagicMock(spec=carla.Location)
        self.mock_map.get_waypoint.return_value = self.mock_state_space
    
    # Initialize D* object with mocked components
        self.d_star = D_star(self.mock_waypoint, None, None, self.mock_vehicle, self.mock_world, self.mock_map)
    
    # Manually set attributes to avoid NoneType errors
        self.d_star.x0 = MagicMock(spec=carla.Waypoint)
        self.d_star.xt = MagicMock(spec=carla.Waypoint)
        self.d_star.start_location = MagicMock(spec=carla.Transform)
        self.d_star.goal_location = MagicMock(spec=carla.Transform)
        self.d_star.state_space = self.mock_state_space
        self.d_star.waypoints = [self.mock_waypoint]  # Provide at least one waypoint to avoid empty list
    
    # Replace the priority queue with a simple instance
        self.d_star.OPEN = PriorityQueue()

    def test_populate_open(self):
        # Mock next waypoint
        self.mock_state_space.next.return_value = [self.mock_waypoint]
        self.d_star.cost = MagicMock(return_value=5.0)  # Mock cost function

        # Call the method
        open_queue = self.d_star.populate_open()

        # Assert the state was added to the priority queue
        self.assertFalse(open_queue.empty(), "OPEN queue should not be empty after populate_open.")
        item = open_queue.get_nowait()  # Get the item from the queue
        self.assertEqual(item[0], 5.0, "The key in the OPEN queue should match the mocked cost value.")
        self.assertEqual(item[1], self.mock_state_space, "The state in the OPEN queue should be the mocked state space.")

if __name__ == '__main__':
    unittest.main()
