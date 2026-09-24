from controllers.controller import Controller
from runtime import Runtime
import json
import os
import time


class FileWriteWatchController(Controller):
    """Watches a directory for NTAG write requests and answers with results.

    The reader belongs to this daemon (root), while the thing that wants to
    program a tag -- AFC in Klipper -- runs as another user and cannot touch
    the SPI bus. A directory both can reach is the channel: AFC drops a request
    file, this picks it up and hands it to the runtime (which owns the reader),
    and the result goes back as a file. No socket, no moonraker round trip.

    Request  req-<token>.json : {"slot": int, "start_page": int, "data": hex}
    Result   res-<token>.json : {"ok": bool, "uid": str, "pages"|"error": ...}

    The request file is removed BEFORE the write runs, so a crash mid-write
    cannot leave a request that re-fires forever. The result is written to a
    .tmp and renamed, so a reader never sees a half-written result.
    """

    def __init__(self, config: dict):
        super().__init__(config)
        self.request_dir = str(config["request_dir"])
        self.poll_interval = float(config.get("poll_interval_seconds", "0.5"))
        self.result_ttl = float(config.get("result_ttl_seconds", "120"))
        self.runtime: Runtime

    def loop(self):
        os.makedirs(self.request_dir, exist_ok=True)
        self.logger.info(f"watching {self.request_dir} for NTAG write requests")
        while True:
            try:
                self._scan_once()
            except Exception as e:
                self.logger.error(f"write-watch error: {e}")
            time.sleep(self.poll_interval)

    def _scan_once(self):
        for name in sorted(os.listdir(self.request_dir)):
            if name.startswith("req-") and name.endswith(".json"):
                token = name[len("req-"):-len(".json")]
                self._handle(token, os.path.join(self.request_dir, name))
            elif name.startswith("res-") and name.endswith(".json"):
                # The requester reads results but cannot delete them (different
                # user, our files), so age them out here.
                self._sweep(os.path.join(self.request_dir, name))

    def _handle(self, token: str, req_path: str):
        try:
            with open(req_path) as f:
                req = json.load(f)
        except Exception as e:
            # An unreadable request must not spin forever.
            self._remove(req_path)
            self._write_result(token, {"ok": False,
                                       "error": f"bad request file: {e}"})
            return

        # Remove first: a request that survives its own write would re-run on
        # every poll.
        self._remove(req_path)

        try:
            slot = int(req["slot"])
            start_page = int(req["start_page"])
            data = bytes.fromhex(req["data"])
        except (KeyError, ValueError, TypeError) as e:
            self._write_result(token, {"ok": False,
                                       "error": f"malformed request: {e}"})
            return

        result = self.runtime.write_ntag(slot, start_page, data)
        self._write_result(token, result)

    def _write_result(self, token: str, result: dict):
        res_path = os.path.join(self.request_dir, f"res-{token}.json")
        tmp_path = res_path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(result, f)
        os.replace(tmp_path, res_path)

    def _sweep(self, path: str):
        try:
            if time.time() - os.path.getmtime(path) > self.result_ttl:
                os.remove(path)
        except OSError:
            pass

    @staticmethod
    def _remove(path: str):
        try:
            os.remove(path)
        except OSError:
            pass
