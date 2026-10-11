import json
from http.server import ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from demo import make_demo
from server import Handler, LocalHTTPServer, ReplayService


class HttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix='lidar_replay_http_')
        cls.http = LocalHTTPServer(('127.0.0.1', 0), Handler)
        cls.http.service = ReplayService()
        cls.http.export_dir = Path(cls.tmp.name)
        cls.worker = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.worker.start()
        cls.base = f'http://127.0.0.1:{cls.http.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.tmp.cleanup()

    def post(self, path, data):
        req = urllib.request.Request(self.base + path, json.dumps(data).encode(), {'Content-Type':'application/json'})
        with urllib.request.urlopen(req, timeout=10) as response:
            return json.load(response)

    def test_export_saved_file_and_reimport_restore_map_and_params(self):
        output = make_demo()
        output['frames'] = output['frames'][:5]
        output.update(params={**self.http.service.defaults, 'min_points': 1},
                      car_width_mm=500, safety_mm=65, compare=True, problem_frame_index=4)
        saved = self.post('/api/export', {'recording':output})
        raw = Path(saved['path']).read_text(encoding='utf-8')
        self.assertEqual(json.loads(raw), output)
        with urllib.request.urlopen(self.base + saved['url']) as response:
            self.assertEqual(response.read().decode('utf-8'), raw)
        imported = self.post('/api/import', {'filename':saved['filename'], 'text':raw})
        for key in ('params', 'car_width_mm', 'safety_mm', 'compare', 'problem_frame_index'):
            self.assertEqual(imported[key], output[key])
        request = {key: imported[key] for key in ('dataset_id','params','car_width_mm','safety_mm','compare')}
        request['index'] = imported['problem_frame_index']
        replayed = self.post('/api/replay', request)
        self.assertEqual(replayed['frame']['scan_num'], 5)
        self.assertTrue(any(road['state']=='blocked' for road in replayed['frame']['map_state']['roads']))

    def test_summary_text_rejected_with_readable_error(self):
        with self.assertRaises(urllib.error.HTTPError) as error:
            self.post('/api/import', {'filename':'stats.txt','text':'SCAN n=1 raw=400 valid=200'})
        self.assertEqual(error.exception.code, 400)
        self.assertIn('统计日志', json.load(error.exception)['error'])

    def test_external_origin_blocked(self):
        req = urllib.request.Request(self.base+'/api/info',headers={'Origin':'https://example.com'})
        with self.assertRaises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(req)
        self.assertEqual(error.exception.code, 403)

    def test_second_server_cannot_share_the_same_port(self):
        with self.assertRaises(OSError):
            LocalHTTPServer(('127.0.0.1', self.http.server_port), Handler)


if __name__ == '__main__':
    unittest.main()
