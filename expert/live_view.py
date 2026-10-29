"""A live bird's-eye view of the NMPC's solves, in a browser: the agent serves a page and pushes each plan to it.

    scripts/run_bench2drive.sh --agent simlingo_nmpc --routes 0 --live     (NMPC_LIVE_PORT, 8765 by default)
    ssh -L 8765:localhost:8765 <this machine>, then open http://localhost:8765

The page shows, in the car's frame (forward up): SimLingo's path and the lane continuation past it, the corridor
the chosen candidate planned in, the plan coloured by speed, the actors with their predictions, the stop lines;
beside it the planned speed against SimLingo's, and the solve times. Each plan is pushed as one JSON message over
Server-Sent Events; a page opened late gets the latest, and reconnects by itself when the next route's process
starts serving. It only reads what the agent computed: nothing here takes part in driving.
"""
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "live_view.html")


class LiveView:
    def __init__(self, port):
        self.port = port
        self._latest = None
        self._seq = 0
        self._cond = threading.Condition()
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # quiet: the leaderboard's log is busy enough
                pass

            def do_GET(self):
                if self.path == "/stream":
                    return view._stream(self)
                body = open(PAGE, "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        ThreadingHTTPServer.allow_reuse_address = True
        ThreadingHTTPServer.daemon_threads = True
        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def _stream(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type", "text/event-stream")
        handler.send_header("Cache-Control", "no-cache")
        handler.end_headers()
        seen = -1
        try:
            while True:
                with self._cond:
                    self._cond.wait_for(lambda: self._seq != seen, timeout=15.0)
                    data, seq = self._latest, self._seq
                if seq == seen or data is None:
                    handler.wfile.write(b": keep-alive\n\n")  # through proxies and tunnels
                else:
                    handler.wfile.write(b"data: " + data + b"\n\n")
                    seen = seq
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def publish(self, snapshot):
        data = json.dumps(snapshot, separators=(",", ":")).encode()
        with self._cond:
            self._latest, self._seq = data, self._seq + 1
            self._cond.notify_all()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def _r(v):
    return round(float(v), 2)


def snapshot(t, path, s_now, s_simlingo, scene, decision):
    """What the page draws for one plan, in the car's frame (metres, y left)."""
    def at(s, e=0.0):
        q = path.pose(s, e)
        return [_r(q.x), _r(q.y)]

    def along(s0, s1, step):
        n = max(1, int((s1 - s0) / step))
        return [s0 + (s1 - s0) * k / n for k in range(n + 1)]

    s_end = min(path.length, s_now + 110.0)
    chosen = next((c for c in decision.candidates if c.label == decision.label), None)
    corridor = []
    if chosen is not None:
        cr = chosen.problem.corridor
        corridor = [[at(s, lo), at(s, hi)] for s, lo, hi in zip(cr.s, cr.lo, cr.hi) if s_now - 10.0 <= s <= s_end]
    plan = decision.plan
    x = plan.x
    actors = []
    for a in scene.actors:
        actors.append({"id": a.id, "kind": str(a.kind).split(".")[-1], "parked": bool(a.parked),
                       "x": _r(a.x[0]), "y": _r(a.y[0]), "psi": round(float(a.psi[0]), 3),
                       "l": _r(a.length), "w": _r(a.width),
                       "traj": [[_r(px), _r(py)] for px, py in list(zip(a.x, a.y))[::2]]})
    stops = []
    for st in scene.stops:
        q = path.pose(st.s)
        stops.append({"kind": str(st.kind).split(".")[-1], "x": _r(q.x), "y": _r(q.y), "psi": round(q.psi, 3),
                      "ahead": _r(st.s - s_now)})
    return {
        "t": round(t, 2),
        "v": _r(scene.ego.v), "v_max": _r(scene.v_max),
        "simlingo": [at(s) for s in along(max(0.0, s_now - 15.0), s_simlingo, 1.0)],
        "beyond": [at(s) for s in along(s_simlingo, s_end, 4.0)],
        "corridor": corridor,
        "plan": [at(s, e) + [_r(v)] for s, e, v in zip(x[:, 0], x[:, 1], x[:, 3])],
        "plan_t": [_r(v) for v in plan.times], "plan_v": [_r(v) for v in x[:, 3]],
        "ref_t": [_r(v) for v in scene.v_ref_t], "ref_v": [_r(v) for v in scene.v_ref],
        "actors": actors, "stops": stops,
        "label": decision.label, "success": bool(plan.success), "ms": round(1e3 * decision.wall_time, 1),
        "candidates": [[c.label, round(c.score, 1) if math.isfinite(c.score) else None,
                        round(1e3 * c.plan.solve_time, 1), bool(c.plan.success)] for c in decision.candidates],
        "notes": list(decision.notes),
    }
