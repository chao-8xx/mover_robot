import copy
import unittest

from demo import make_demo
from recording import RecordingError
from server import ReplayService


class ServiceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.service = ReplayService()

    def setUp(self):
        self.dataset = self.service.register(make_demo())
        self.request = {'dataset_id': self.dataset['dataset_id'], 'index': 0,
                        'params': copy.deepcopy(self.service.defaults), 'compare': True}

    def test_backward_seek_equals_fresh_full_history(self):
        request = self.request
        for index in (0, 6, 20, 3, 39, 15):
            request['index'] = index
            got = self.service.replay(request)
            fresh_request = {**request, 'dataset_id': self.service.register(make_demo())['dataset_id']}
            expected = self.service.replay(fresh_request)
            self.assertEqual(got['frame'], expected['frame'])
            self.assertEqual(got['baseline'], expected['baseline'])

    def test_default_comparison_and_parameter_isolation(self):
        self.request['index'] = 16
        self.request['params']['mask_enabled'] = not self.service.defaults['mask_enabled']
        result = self.service.replay(self.request)
        baseline = self.service.replay({**self.request, 'params': self.service.defaults})
        self.assertEqual(result['baseline'], baseline['frame'])
        self.assertNotEqual(result['frame']['counts'], result['baseline']['counts'])

    def test_global_pose_invalid_stops_map(self):
        self.request.update(index=6, pose_override={'x': 1200, 'y': 800, 'yaw_deg': 0, 'valid': False})
        result = self.service.replay(self.request)['frame']
        self.assertFalse(result['map_state']['updated'])
        self.assertTrue(all(road['state'] == 'unknown' for road in result['map_state']['roads']))

    def test_bad_parameters_fail_without_corrupting_cache(self):
        self.request['params']['min_distance'] = 9000
        with self.assertRaises(RecordingError):
            self.service.replay(self.request)
        self.request['params'] = self.service.defaults
        self.assertEqual(self.service.replay(self.request)['index'], 0)


if __name__ == '__main__':
    unittest.main()
