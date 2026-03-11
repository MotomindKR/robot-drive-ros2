#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from urllib.parse import urlparse, parse_qs
import json
import os
import subprocess
from std_msgs.msg import String
import math
from geometry_msgs.msg import PoseWithCovarianceStamped 
import re
from std_msgs.msg import String as MapNameMsg
import time

def occupancy_grid_to_dict(msg):
    return {
        'header': {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec
            },
            'frame_id': msg.header.frame_id
        },
        'info': {
            'map_load_time': {
                'sec': msg.info.map_load_time.sec,
                'nanosec': msg.info.map_load_time.nanosec
            },
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': {
                'position': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'orientation': {
                    'x': msg.info.origin.orientation.x,
                    'y': msg.info.origin.orientation.y,
                    'z': msg.info.origin.orientation.z,
                    'w': msg.info.origin.orientation.w
                }
            }
        },
        'data': list(msg.data)
    }

def transform_to_dict(msg):
    return {
        'header': {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec
            },
            'frame_id': msg.header.frame_id
        },
        'child_frame_id': msg.child_frame_id,
        'transform': {
            'translation': {
                'x': msg.transform.translation.x,
                'y': msg.transform.translation.y,
                'z': msg.transform.translation.z
            },
            'rotation': {
                'x': msg.transform.rotation.x,
                'y': msg.transform.rotation.y,
                'z': msg.transform.rotation.z,
                'w': msg.transform.rotation.w
            }
        }
    }

class TopicListHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200, "OK")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        node = self.server.ros_node
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)

        # 0) 현재 map_name 반환
        if path == '/mapname':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(
                json.dumps({'mapname': node.current_mapname}).encode('utf-8')
            )
            return

        # 1) Start mapping (requires ?mapname=...)
        if path == '/run_mapping':
            # mapname 파라미터 추출
            mapnames = query.get('mapname')
            if not mapnames or not mapnames[0]:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameter: mapname'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            # 사용자가 제공한 mapname 인자를 스크립트에 전달
            script = os.path.expanduser('~/run_mapping.sh')
            try:
                subprocess.Popen(['bash', script, mapname])
                # 성공 응답
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'mapping_started',
                    'script': script,
                    'mapname': mapname
                }).encode('utf-8'))
            except Exception as e:
                # 실패 응답
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': str(e)
                }).encode('utf-8'))
            return

        if path == '/stop_autodrive':
            script = os.path.expanduser('~/terminate_autodrive.sh')
            try:
                subprocess.Popen(['bash', script])
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'autodrive_stopped',
                    'script': script
                }).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return
        

        # 2) Stop mapping
        if path == '/stop_mapping':
            script = os.path.expanduser('~/terminate_mapping.sh')
            try:
                subprocess.Popen(['bash', script])
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'mapping_stopped',
                    'script': script
                }).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return

        # 3) Save mapping (requires ?mapname=...)
        if path == '/save_mapping':
            mapnames = query.get('mapname')
            if not mapnames or not mapnames[0]:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameter: mapname'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            script = os.path.expanduser('~/map_saver.sh')
            try:
                subprocess.Popen(['bash', script, mapname])
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'save_invoked',
                    'script': script,
                    'mapname': mapname
                }).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return

        # 4) List saved .data files
        if path == '/mapFileList':
            maps_dir = os.path.expanduser('~/ello_ws/src/od2_navigation/maps')
            try:
                files = [
                    f for f in os.listdir(maps_dir)
                    if f.endswith('.data') and os.path.isfile(os.path.join(maps_dir, f))
                ]
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'maps': files}).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return

        # 5) Run autodrive (requires ?mapname=...)
        if path == '/run_autodrive':
            mapnames = query.get('mapname')
            if not mapnames or not mapnames[0]:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameter: mapname'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            script = os.path.expanduser('~/run_autodrive.sh')
            try:
                subprocess.Popen(['bash', script, mapname])
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'status': 'autodrive_started',
                    'script': script,
                    'mapname': mapname
                }).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return

        # --- 기존 ROS / topic viewer endpoints ---
                # 6) Robot node/edge 정보 반환
        if path == '/robot_node_edge':
            mapname = query.get('mapname', ['home'])[0] or 'home'
            if not re.fullmatch(r'[A-Za-z0-9_\-]+', mapname):
                self.send_response(400)
                ...
                self.wfile.write(json.dumps({'error': 'Invalid mapname'}).encode())
                return
            config_path = os.path.expanduser(
                f'~/ello_ws/src/od2_node_manager/config/{mapname}.json'
            )
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(data).encode('utf-8'))
            except FileNotFoundError:
                self.send_response(404)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Config file not found: {config_path}'
                }).encode('utf-8'))
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({'error': str(e)}).encode('utf-8'))
            return


        # 6) Root: list ROS2 topics
        if path == '/':
            topics = node.get_topic_names_and_types()
            html = "<html><head><meta charset='utf-8'><title>ROS2 Topics</title></head><body><h1>ROS2 Topics List</h1><ul>"
            for topic, types in topics:
                html += f"<li><strong>{topic}</strong>: {', '.join(types)}</li>"
            html += "</ul></body></html>"
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(html.encode('utf-8'))
            return
                # 6) 노드 추가 (requires ?mapname=&x=&y=&theta=)
        if path == '/add_node':
            # 쿼리 파라미터 확보
            mapnames = query.get('mapname')
            xs = query.get('x')
            ys = query.get('y')
            thetas = query.get('theta')

            # 필수 파라미터 검증
            if not (mapnames and xs and ys and thetas):
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameters: mapname, x, y, theta'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            try:
                x = float(xs[0])
                y = float(ys[0])
                theta = float(thetas[0])
            except ValueError:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Invalid numeric values for x, y or theta'
                }).encode('utf-8'))
                return

            # 설정 파일 경로
            config_dir = os.path.expanduser('~/ello_ws/src/od2_node_manager/config')
            config_path = os.path.join(config_dir, f"{mapname}.json")

            # 파일 존재 확인
            if not os.path.isfile(config_path):
                self.send_response(404)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Config file not found: {config_path}'
                }).encode('utf-8'))
                return

            # JSON 로드
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to read config: {e}'
                }).encode('utf-8'))
                return

            # 새 ID 계산 (기존 노드 중 최대값 + 1)
            existing_ids = [n.get('id', 0) for n in data.get('nodes', [])]
            new_id = max(existing_ids + [0]) + 1

            # 새 노드 객체
            new_node = {
                "id": new_id,
                "name": str(new_id),
                "x": x,
                "y": y,
                "z": 0,
                "theta": theta,
                "type": "default",
                "description": f"This is {new_id}"
            }

            # 노드 리스트에 추가
            data.setdefault('nodes', []).append(new_node)

            # 파일에 덮어쓰기
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to write config: {e}'
                }).encode('utf-8'))
                return

            # 성공 응답
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'node_added',
                'mapname': mapname,
                'node': new_node
            }).encode('utf-8'))
            return
        # 7) 엣지 추가 (requires ?mapname=&start=&end=)
        if path == '/add_edge':
            mapnames = query.get('mapname')
            starts   = query.get('start')
            ends     = query.get('end')
            if not (mapnames and starts and ends):
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameters: mapname, start, end'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            try:
                start = int(starts[0])
                end   = int(ends[0])
            except ValueError:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Invalid integer values for start or end'
                }).encode('utf-8'))
                return

            config_dir  = os.path.expanduser('~/ello_ws/src/od2_node_manager/config')
            config_path = os.path.join(config_dir, f"{mapname}.json")
            if not os.path.isfile(config_path):
                self.send_response(404)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Config file not found: {config_path}'
                }).encode('utf-8'))
                return

            # 로드
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to read config: {e}'
                }).encode('utf-8'))
                return

            # 엣지 추가
            data.setdefault('edges', []).append({'start': start, 'end': end})

            # 덮어쓰기
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to write config: {e}'
                }).encode('utf-8'))
                return

            # 성공 응답
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'edge_added',
                'mapname': mapname,
                'edge': {'start': start, 'end': end}
            }).encode('utf-8'))
            return

        # 8) 노드 삭제 (requires ?mapname=&id=)
        if path == '/delete_node':
            mapnames = query.get('mapname')
            ids      = query.get('id')
            if not (mapnames and ids):
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameters: mapname, id'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            try:
                nid = int(ids[0])
            except ValueError:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Invalid integer value for id'
                }).encode('utf-8'))
                return

            config_path = os.path.join(
                os.path.expanduser('~/ello_ws/src/od2_node_manager/config'),
                f"{mapname}.json"
            )
            if not os.path.isfile(config_path):
                self.send_response(404)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Config file not found: {config_path}'
                }).encode('utf-8'))
                return

            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to read config: {e}'
                }).encode('utf-8'))
                return

            # 노드 필터링 & 연관 엣지 제거
            data['nodes'] = [n for n in data.get('nodes', []) if n.get('id') != nid]
            data['edges'] = [
                e for e in data.get('edges', [])
                if not (e.get('start') == nid or e.get('end') == nid)
            ]

            # 덮어쓰기
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to write config: {e}'
                }).encode('utf-8'))
                return

            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'node_deleted',
                'mapname': mapname,
                'id': nid
            }).encode('utf-8'))
            return

        # 9) 엣지 삭제 (requires ?mapname=&start=&end=)
        if path == '/delete_edge':
            mapnames = query.get('mapname')
            starts   = query.get('start')
            ends     = query.get('end')
            if not (mapnames and starts and ends):
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing required query parameters: mapname, start, end'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            try:
                start = int(starts[0]); end = int(ends[0])
            except ValueError:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Invalid integer values for start or end'
                }).encode('utf-8'))
                return

            config_path = os.path.join(
                os.path.expanduser('~/ello_ws/src/od2_node_manager/config'),
                f"{mapname}.json"
            )
            if not os.path.isfile(config_path):
                self.send_response(404)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Config file not found: {config_path}'
                }).encode('utf-8'))
                return

            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to read config: {e}'
                }).encode('utf-8'))
                return

            # 해당 엣지만 제거
            data['edges'] = [
                e for e in data.get('edges', [])
                if not (e.get('start') == start and e.get('end') == end)
            ]

            # 덮어쓰기
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2)
            except Exception as e:
                self.send_response(500)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to write config: {e}'
                }).encode('utf-8'))
                return

            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'edge_deleted',
                'mapname': mapname,
                'edge': {'start': start, 'end': end}
            }).encode('utf-8'))
            return

        # 7) /map → OccupancyGrid JSON
        if path == '/map':
            # 현재 실행 중인 노드 목록 조회
            # get_node_names_and_namespaces() 는 [(name, namespace), ...] 형태로 반환합니다.
            running = [n for (n, ns) in node.get_node_names_and_namespaces()]

            # slam_toolbox 노드가 없으면 빈 데이터
            if 'slam_toolbox' not in running:
                body = { "info": None, "data": [] }
            else:
                msg = node.latest_messages.get('/map')
                if msg is None:
                    body = { "info": None, "data": [] }
                else:
                    body = occupancy_grid_to_dict(msg)

            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(body).encode('utf-8'))
            return

        # 8) /current_pose → TransformStamped JSON
        if path == '/current_pose':
            msg = node.latest_messages.get('/current_pose')
            if msg is None:
                body = {"error": "No data received yet for topic /current_pose"}
            else:
                body = transform_to_dict(msg)
            self.send_response(200)
            self.send_header('Content-type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(body).encode('utf-8'))
            return

        # 9) /scan_filtered and /cmd_vel → plain text latest message
        if path in ['/scan_filtered', '/cmd_vel']:
            msg = node.latest_messages.get(path)
            body = str(msg) if msg is not None else f"No data received yet for topic {path}"
            self.send_response(200)
            self.send_header('Content-type', 'text/plain; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body.encode('utf-8'))
            return

        # 10) /set_cmd?type=...&value=...
        if path.startswith('/set_cmd'):
            cmd_type = query.get('type', [None])[0]
            value = query.get('value', [None])[0]
            if cmd_type and value:
                try:
                    raw = float(value)
                except ValueError:
                    raw = 0.0
                raw = max(-100.0, min(100.0, raw))
                scaled = (raw / 100.0) * 0.6
                if cmd_type == "linear":
                    node.current_twist.linear.x = scaled
                elif cmd_type == "angular":
                    node.current_twist.angular.z = scaled
                node.cmd_vel_pub.publish(node.current_twist)
                node.latest_messages['/cmd_vel'] = node.current_twist
                response = f"cmd_vel set: {node.current_twist}"
            else:
                response = "Missing parameters in set_cmd"
            self.send_response(200)
            self.send_header('Content-type', 'text/plain; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(response.encode('utf-8'))
            return
        # --- 11) goto_node: 서버사이드에서 노드 이동 명령 전송 ---
        if path == '/goto_node':
            mapnames = query.get('mapname')
            ids = query.get('id')
            if not (mapnames and ids and ids[0].isdigit()):
                self.send_response(400)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing or invalid parameters: mapname, id'
                }).encode('utf-8'))
                return

            mapname = mapnames[0]
            node_id = int(ids[0])

            # ROS 토픽으로 이동 명령 publish
            payload = { 'mapname': mapname, 'id': node_id }
            msg = String()
            msg.data = json.dumps(payload)
            node.get_logger().info(f"goto_node 요청: {payload}")
            node.task_command_pub.publish(msg)

            # HTTP 응답
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'goto_invoked',
                'mapname': mapname,
                'id': node_id
            }).encode('utf-8'))
            return
        # --- 11) relocate: 초기 위치 강제 지정 --------------------------
        if path == '/relocate':
            # ── ① 파라미터 파싱 ───────────────────────────────────────
            mapname = query.get('mapname', [None])[0] or 'unknown'
            xs  = query.get('x',      [None])[0]
            ys  = query.get('y',      [None])[0]
            ths = query.get('theta',  [None])[0]

            try:
                x = float(xs);  y = float(ys);  theta = float(ths)
            except (TypeError, ValueError):
                self.send_response(400)
                self.send_header('Content-Type',
                                 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': 'Missing or invalid parameters: mapname, x, y, theta'
                }).encode('utf-8'))
                return

            # ── ② publish_initial_pose 호출 ──────────────────────────
            try:
                node.publish_initial_pose(x, y, theta)
            except Exception as e:
                node.get_logger().error(f"publish_initial_pose 실패: {e}")
                self.send_response(500)
                self.send_header('Content-Type',
                                 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps({
                    'error': f'Failed to publish initial pose: {e}'
                }).encode('utf-8'))
                return

            # ── ③ 200 OK 응답 ─────────────────────────────────────────
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'status': 'relocate_invoked',
                'mapname': mapname,
                'x': x, 'y': y, 'theta': theta
            }).encode('utf-8'))
            return


        # 404 Not Found
        self.send_error(404, "File Not Found")


class TopicListServer(Node):
    def __init__(self):
        super().__init__('topic_list_server')
        self.get_logger().info("Topic List Server 노드 시작")

        # 각 토픽의 최신 메시지 저장용
        self.latest_messages = {
            '/scan_filtered': None,
            '/map': None,
            '/cmd_vel': None,
            '/current_pose': None
        }
        self.task_command_pub = self.create_publisher(String, 'task_command', 10)
        self.get_logger().info("Publisher 'task_command' 준비 완료")

        self.initial_pose_pub = self.create_publisher(            # ← 추가
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.get_logger().info("Publisher '/initialpose' 준비 완료")
        # 구독자 생성
        self.create_subscription(LaserScan, '/scan_filtered', self.scan_filtered_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(TransformStamped, '/current_pose', self.current_pose_callback, 10)

        # cmd_vel 퍼블리셔 및 초기 메시지
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_twist = Twist()
        self.current_twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        self.current_twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        # map_name 토픽 구독
        self.current_mapname = ''
        self.create_subscription(
            MapNameMsg,
            '/map_name',
            self.map_name_callback,
            10
        )
        self.get_logger().info("Subscriber '/map_name' 준비 완료")

        self.last_map_time = 0.0


        # HTTP 서버 초기화 및 실행
        self.http_server = HTTPServer(('0.0.0.0', 8000), TopicListHTTPRequestHandler)
        self.http_server.ros_node = self
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()
        self.get_logger().info("HTTP 서버가 포트 8000에서 실행 중")

    def scan_filtered_callback(self, msg):
        self.latest_messages['/scan_filtered'] = msg

    def map_callback(self, msg):
        self.latest_messages['/map'] = msg
        self.last_map_time = time.monotonic()

    def cmd_vel_callback(self, msg):
        self.latest_messages['/cmd_vel'] = msg

    def current_pose_callback(self, msg):
        self.latest_messages['/current_pose'] = msg


    def map_name_callback(self, msg: MapNameMsg):
        # map_name_publisher.py 가 발행하는 맵 이름을 저장
        self.current_mapname = msg.data
        self.get_logger().debug(f"map_name 수신: {self.current_mapname}")

    def run_server(self):
        try:
            self.http_server.serve_forever()
        except Exception as e:
            self.get_logger().error(f"HTTP 서버 실행 중 오류 발생: {e}")

    def destroy_node(self):
        self.get_logger().info("HTTP 서버 종료 중...")
        self.http_server.shutdown()
        self.server_thread.join()
        super().destroy_node()
    # ------------------------------------------------------------------
    #  초기 위치(POSE WITH COVARIANCE) 퍼블리시: /initialpose
    # ------------------------------------------------------------------
    def publish_initial_pose(self, x: float, y: float, theta: float):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0

        pose.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.pose.orientation.w = math.cos(theta / 2.0)

        # 0.25 m² · 15°(≈0.068 rad²)
        pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.068
        ]

        self.initial_pose_pub.publish(pose)
        self.get_logger().info(
            f"/initialpose 게시 완료: x={x:.2f}, y={y:.2f}, θ={theta:.3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TopicListServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
