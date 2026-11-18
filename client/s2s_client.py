"""HTTP client for the s2s source-world server (the agent's side; needs only numpy and requests).

    env = SourceWorldClient("https://<host>", token="...")     # or S2S_SERVER_URL / S2S_TOKEN
    env.info()                       # your seeds, budgets, cameras, primitives, workspace
    env.reset("dev_s0003")           # starts an episode (blocks until the world is ready)
    calib = env.get_calibration()    # camera intrinsics/extrinsics
    obs = env.observe("cam_0")       # {"rgb", "depth", "t", "meta"}
    res = env.execute("push", {"contact_point": [...], "direction": [1, 0],
                               "distance": 0.03, "speed": 0.02})

reset() and execute() can take minutes. The server runs them as jobs (202 + a job id)
and this client polls GET /v1/jobs/<id> until the result is ready, so for the caller
they are ordinary blocking calls. A dropped connection while polling is retried: the
result stays on the server and is never charged twice.
"""

import io
import json
import os
import time

import numpy as np
import requests

POLL_WAIT_S = 50          # server-side long poll per request (server caps it at 60 s)
REQUEST_TIMEOUT_S = 90    # per HTTP request; stays under proxy limits (Cloudflare ~100 s)
POLL_RETRIES = 20


class ServerError(RuntimeError):
    def __init__(self, status, message):
        super().__init__(f"{status}: {message}")
        self.status = status


class SourceWorldClient:
    def __init__(self, url=None, token=None, job_timeout=3600):
        self.url = (url or os.environ["S2S_SERVER_URL"]).rstrip("/")
        token = token or os.environ["S2S_TOKEN"]
        self.s = requests.Session()
        self.s.headers["Authorization"] = f"Bearer {token}"
        self.job_timeout = job_timeout

    @staticmethod
    def _decode(r):
        if r.status_code != 200:
            try:
                msg = r.json().get("detail") or r.json().get("error")
            except ValueError:
                msg = r.text[:500]
            raise ServerError(r.status_code, msg)
        if r.headers.get("content-type", "").startswith("application/json"):
            return r.json()
        with np.load(io.BytesIO(r.content), allow_pickle=False) as z:
            out = {k: z[k] for k in z.files if k != "meta"}
            out["meta"] = json.loads(str(z["meta"]))
        return out

    def _call(self, method, path, body=None):
        r = self.s.request(method, self.url + path, json=body, timeout=REQUEST_TIMEOUT_S)
        if r.status_code == 202 and r.headers.get("content-type", "").startswith("application/json"):
            return self._wait(r.json()["job"])
        return self._decode(r)

    def _wait(self, job):
        t0, failures = time.time(), 0
        while time.time() - t0 < self.job_timeout:
            try:
                r = self.s.get(f"{self.url}/v1/jobs/{job}", params={"wait": POLL_WAIT_S}, timeout=REQUEST_TIMEOUT_S)
            except requests.RequestException:
                failures += 1
                if failures > POLL_RETRIES:
                    raise
                time.sleep(min(30, 2 ** failures))
                continue
            failures = 0
            if r.status_code == 202:
                continue
            return self._decode(r)
        raise ServerError(504, f"job {job} did not finish within {self.job_timeout} s")

    def info(self):
        return self._call("GET", "/v1/info")

    def budget(self):
        return self._call("GET", "/v1/budget")

    def reset(self, seed):
        return self._call("POST", "/v1/reset", {"seed": seed})

    def get_calibration(self):
        return self._call("GET", "/v1/calibration")

    def observe(self, camera="cam_0"):
        return self._call("POST", "/v1/observe", {"camera": camera})

    def execute(self, primitive, params=None):
        return self._call("POST", "/v1/execute", {"primitive": primitive, "params": params or {}})
