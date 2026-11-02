#!/usr/bin/env python3
"""PAI final project: three-lane AI stopping with pre-drawn guide lines.

Only the three MOVING turtles remain visible. Each lane has a thin pale
pre-drawn visual guide and a short orange line at the virtual barrier X=8.5.
The turtle paints a separate coloured trail as it moves. These lines are
VISUAL AIDS ONLY: the AI uses /pose.x and the fixed virtual barrier coordinate,
not image perception or line following. No physical proximity sensor/collision.
Original programs and the ROS 2 installation are not modified.
"""
import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')  # Ubuntu 22.04 NumPy/SciPy/sklearn

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

LANES = (
    ('A', 'turtle1', 2.0, 8.4, (249, 231, 86)),
    ('B', 'pai_lane_B', 3.5, 5.55, (107, 232, 157)),
    ('C', 'pai_lane_C', 5.0, 2.7, (255, 168, 204)),
)


def train_model():
    from sklearn.tree import DecisionTreeClassifier
    examples = [[n / 20.0] for n in range(101)]
    labels = [int(x[0] > 1.0) for x in examples]
    tree = DecisionTreeClassifier(max_depth=1, random_state=0)
    tree.fit(examples, labels)
    assert tree.predict([[0.5], [2.0]]).tolist() == [0, 1]
    return tree


def main():
    p = argparse.ArgumentParser(description='Sequential three-lane ROS 2 AI stop demonstration')
    p.add_argument('--speed', type=float, default=0.17,
                   help='Movement speed, default 0.17 (roughly one quarter of original 0.65)')
    p.add_argument('--barrier-x', type=float, default=8.5)
    p.add_argument('--self-test', action='store_true')
    p.add_argument('--prelude', type=float, default=12)
    p.add_argument('--between', type=float, default=5)
    p.add_argument('--final-pause', type=float, default=20)
    p.add_argument('--guide-colour', choices=('pale', 'white'), default='pale',
                   help='Pre-drawn reference line colour (not an AI input)')
    args = p.parse_args()
    model = train_model()
    if args.self_test:
        import numpy, scipy, sklearn
        print('NumPy:', numpy.__version__, 'SciPy:', scipy.__version__, 'scikit-learn:', sklearn.__version__)
        print('Decision tree threshold:', round(float(model.tree_.threshold[0]), 3))
        print('Distance 0.5 => STOP, 2.0 => MOVE: OK')
        print('Only three turtles; guidance lines are visual only, not AI inputs.')
        print('=== final result ===\n【【【【OK！】】】】')
        return 0
    if not (0.0 < args.speed <= 1.0 and 8.0 <= args.barrier_x <= 9.0):
        print('=== final result ===\n【【【【NG：確認が必要】】】】')
        return 2

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from turtlesim.msg import Pose
    from turtlesim.srv import Spawn, TeleportAbsolute, SetPen, Kill
    from std_srvs.srv import Empty

    class ThreeLaneDemo(Node):
        def __init__(self):
            super().__init__('pai_3lane_ai_controller')
            self.poses = {}
            self.pubs = {}
            self.results = []
            self.spawn_client = self.create_client(Spawn, '/spawn')
            self.kill_client = self.create_client(Kill, '/kill')
            self.clear_client = self.create_client(Empty, '/clear')
            for label, name, x, y, color in LANES:
                self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
                self.create_subscription(Pose, f'/{name}/pose',
                                         lambda msg, name=name: self.poses.__setitem__(name, msg), 10)
            self.path = Path('/workspace/logs')
            self.path.mkdir(parents=True, exist_ok=True)
            self.run_id = datetime.now().strftime('%Y%m%d_%H%M%S')

        def call(self, client, request, label, required=True):
            if not client.wait_for_service(timeout_sec=5.0):
                if required:
                    raise RuntimeError(f'{label}: service not available')
                print(f'[INFO] {label}: service unavailable; skipping')
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
            if not future.done() or future.exception() is not None:
                if required:
                    raise RuntimeError(f'{label}: service call failed')
                print(f'[INFO] {label}: service call skipped')
                return None
            return future.result()

        def set_pen(self, name, enabled, rgb=(255, 255, 255), width=4):
            req = SetPen.Request()
            req.r, req.g, req.b = rgb
            req.width = width
            req.off = 0 if enabled else 1
            client = self.create_client(SetPen, f'/{name}/set_pen')
            self.call(client, req, f'{name}/set_pen')
            self.destroy_client(client)

        def move_to(self, name, x, y):
            req = TeleportAbsolute.Request()
            req.x, req.y, req.theta = float(x), float(y), 0.0
            client = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
            self.call(client, req, f'{name}/teleport_absolute')
            self.destroy_client(client)
            self.poses.pop(name, None)

        def spawn_if_absent(self, name, x, y):
            test_client = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
            present = test_client.wait_for_service(timeout_sec=0.5)
            self.destroy_client(test_client)
            if present:
                return
            req = Spawn.Request()
            req.x, req.y, req.theta, req.name = float(x), float(y), 0.0, name
            response = self.call(self.spawn_client, req, f'spawn {name}')
            if response.name != name:
                raise RuntimeError(f'spawn name mismatch for {name}: {response.name}')

        def speed(self, name, value):
            msg = Twist()
            msg.linear.x = float(value)
            self.pubs[name].publish(msg)

        def pause(self, seconds):
            until = time.monotonic() + max(0, seconds)
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, until-time.monotonic())))

        def remove_if_present(self, name):
            # Remove ONLY markers created by the previous PAI experiment.
            client = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
            present = client.wait_for_service(timeout_sec=0.3)
            self.destroy_client(client)
            if present:
                req = Kill.Request()
                req.name = name
                self.call(self.kill_client, req, f'remove old marker {name}')
                self.pause(0.10)

        def paint_segment(self, name, x1, y1, x2, y2, rgb, width):
            # In ROS 2 Humble, teleport_absolute draws a straight stroke if pen is ON.
            # Pauses ensure the GUI processes each queued teleport before pen changes.
            self.set_pen(name, False)
            self.move_to(name, x1, y1)
            self.pause(0.20)
            self.set_pen(name, True, rgb=rgb, width=width)
            self.pause(0.10)
            self.move_to(name, x2, y2)
            self.pause(0.20)
            self.set_pen(name, False)
            self.pause(0.10)

        def prepare(self):
            for obsolete in ('virtual_obstacle', 'pai_marker_A',
                             'pai_marker_B', 'pai_marker_C'):
                self.remove_if_present(obsolete)
            # Never delete one of the three actual moving turtles.
            for label, name, x, y, rgb in LANES:
                self.spawn_if_absent(name, x, y)
                self.speed(name, 0.0)
                self.set_pen(name, False)
                self.move_to(name, x, y)
                self.pause(0.20)

            # Erase old drawing trails. This does NOT remove turtles or change files.
            self.call(self.clear_client, Empty.Request(), 'clear old trails')
            self.pause(0.15)

            # A guide finishes at the nominal stop line, ~1 unit before the
            # virtual barrier. Real stop location may vary slightly by timestep.
            nominal_stop_x = args.barrier_x - float(model.tree_.threshold[0])
            guide_rgb = (195, 210, 229) if args.guide_colour == 'pale' else (245, 245, 245)
            for label, name, x, y, rgb in LANES:
                self.paint_segment(name, x, y, nominal_stop_x, y,
                                   guide_rgb, width=2)
                self.paint_segment(name, args.barrier_x, y-0.30,
                                   args.barrier_x, y+0.30,
                                   (244, 159, 80), width=7)
                self.move_to(name, x, y)
                self.pause(0.20)
                self.speed(name, 0.0)
                self.set_pen(name, True, rgb=rgb, width=5)
                self.pause(0.15)

            print('[GUIDE] All three thin guides are drawn before AI-controlled movement.', flush=True)
            print('[GUIDE] Orange short vertical lines show the VIRTUAL barrier X coordinate.', flush=True)
            print('[AI] No line sensing: decisions use the numerical distance from /pose.x.', flush=True)
            self.pause(0.30)

        def run_lane(self, label, name, start_x, y, color):
            print(f'\n========== TRIAL {label} : START X={start_x}, Y={y} ==========')
            file = self.path / f'video_guides_{label}_{self.run_id}.csv'
            start = time.monotonic()
            last_report = -2.0
            success = False
            final_distance = None
            with file.open('w', newline='', encoding='utf-8') as out:
                writer = csv.writer(out)
                writer.writerow(['elapsed_sec','lane','x','y','virtual_distance','ai_action','command_speed'])
                while rclpy.ok() and (time.monotonic() - start) < 42:
                    rclpy.spin_once(self, timeout_sec=0.08)
                    pose = self.poses.get(name)
                    if pose is None:
                        continue
                    elapsed = time.monotonic()-start
                    x = float(pose.x)
                    distance = max(0.0, args.barrier_x-x)
                    action = int(model.predict([[distance]])[0])
                    value = args.speed if action else 0.0
                    self.speed(name, value)
                    writer.writerow([f'{elapsed:.3f}',label,f'{x:.4f}',f'{float(pose.y):.4f}',
                                     f'{distance:.4f}', 'MOVE' if action else 'STOP', f'{value:.3f}'])
                    if elapsed-last_report >= 1.5 or not action:
                        print(f'{label} t={elapsed:5.1f}s x={x:5.2f} distance={distance:4.2f} '
                              f'AI={"MOVE" if action else "STOP"}', flush=True)
                        last_report = elapsed
                    if not action:
                        final_distance = distance
                        success = 0.0 < distance <= 1.15
                        print(f'RESULT {label}: {"SUCCESS" if success else "FAILED"}; '
                              f'stop distance={distance:.3f}; log={file.name}', flush=True)
                        break
            self.speed(name, 0.0)
            if final_distance is None:
                print(f'RESULT {label}: FAILED (timeout or no position data)', flush=True)
            self.results.append((label, start_x, final_distance, success, file.name))
            self.pause(0.5)  # give the simulator time to receive zero-velocity command
            return success

        def show_summary(self):
            # Keep the three results together and visible on the right at the end.
            print('\033[2J\033[H', end='')
            print('==============================================')
            print(' PHYSICAL AI FINAL PROJECT / ROS 2 turtlesim')
            print(' THREE-LANE AI STOPPING (VISUAL GUIDES / NO MARKER TURTLES)')
            print('==============================================')
            print(f'DecisionTreeClassifier threshold: {model.tree_.threshold[0]:.3f}')
            print(f'Original speed: 0.650 | Current speed: {args.speed:.3f}')
            print(f'Virtual barrier X={args.barrier_x:.2f}; no physical sensor/collision')
            print('Guide lines are visual only: AI uses x coordinate, not line-following.')
            for label, x, distance, success, filename in self.results:
                d = f'{distance:.3f}' if distance is not None else 'N/A'
                print(f'TRIAL {label}  START_X={x:.1f}  STOP_DIST={d}  '
                      f'{"SUCCESS" if success else "FAILED"}')
            ok = len(self.results) == 3 and all(r[3] for r in self.results)
            print('----------------------------------------------')
            print('=== final result ===')
            print('【【【【OK！】】】】' if ok else '【【【【NG：確認が必要】】】】')
            print('----------------------------------------------')
            return ok

    rclpy.init()
    demo = ThreeLaneDemo()
    passed = False
    try:
        demo.prepare()
        print('==============================================')
        print(' AI STOP / THREE LANES: A(top), B(mid), C(bottom)')
        print(' Pre-drawn path guides and orange virtual barrier strokes.')
        print(' Three turtles are visible from the start.')
        print(f' Speed {args.speed:.3f}; fixed virtual barrier X={args.barrier_x:.1f}')
        print(' AI learns distance <= 1.0 => STOP, otherwise MOVE.')
        print('==============================================')
        demo.pause(args.prelude)
        for i, (label, name, x, y, color) in enumerate(LANES):
            demo.run_lane(label, name, x, y, color)
            if i != len(LANES)-1:
                demo.pause(args.between)
        passed = demo.show_summary()
        demo.pause(args.final_pause)
    except KeyboardInterrupt:
        print('\n[INFO] Recording interrupted by Ctrl+C')
    except Exception as exc:
        print(f'[NG] {type(exc).__name__}: {exc}')
    finally:
        for _, name, *_ in LANES:
            demo.speed(name, 0.0)
        demo.pause(0.25)
        demo.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
