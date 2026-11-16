"""Bounded best-effort image diagnostics, never trajectory qualification."""
import copy
import json
import os
from pathlib import Path
import queue
import threading

import cv2


class DiagnosticWriter:
    def __init__(self, directory, image_writer=None):
        self.directory = Path(directory)
        self.image_writer = image_writer or cv2.imwrite
        self.pending = queue.Queue(maxsize=2)
        self.stop = threading.Event()
        self.dropped = 0
        self.errors = 0
        self.worker = threading.Thread(target=self._run, name='camera_diagnostics', daemon=True)
        self.worker.start()

    def _submit(self, job):
        if self.stop.is_set():
            self.dropped += 1
            return False
        try:
            self.pending.put_nowait(job)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def overlay(self, image, raw):
        return self._submit(('overlay', image.copy(), None if raw is None else raw.copy()))

    def rejection(self, event, before, after, packet):
        return self._submit(('rejection', str(event),
                             None if before is None else before.copy(),
                             None if after is None else after.copy(), copy.deepcopy(packet)))

    def packet(self, packet):
        # Post-publication, bounded and best effort; records actual timestamps,
        # including rejected frames. A dropped diagnostic is not a guide loss.
        return self._submit(('packet', copy.deepcopy(packet)))

    def _atomic_image(self, name, image, options):
        temporary = self.directory / ('.' + str(os.getpid()) + '_' + name)
        if not self.image_writer(str(temporary), image, options):
            raise OSError('Diagnostic image write failed')
        temporary.replace(self.directory/name)

    def _write(self, job):
        if job[0] == 'packet':
            with (self.directory/'guide_packets.jsonl').open('a') as log:
                log.write(json.dumps(job[1], allow_nan=False)+'\n')
        elif job[0] == 'overlay':
            self._atomic_image('near_overlay.png', job[1], [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if job[2] is not None:
                self._atomic_image('near_latest_raw.jpg', job[2], [cv2.IMWRITE_JPEG_QUALITY, 90])
        else:
            _, event, before, after, packet = job
            for suffix, frame in [('_before.jpg', before), ('_after.jpg', after)]:
                if frame is not None and not self.image_writer(event+suffix, frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                    raise OSError('Diagnostic rejection image write failed')
            temporary = Path(event+'.json.tmp')
            temporary.write_text(json.dumps(packet, allow_nan=False))
            temporary.replace(Path(event+'.json'))

    def _run(self):
        while not self.stop.is_set() or not self.pending.empty():
            try:
                job = self.pending.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self._write(job)
            except Exception:
                # Diagnostics must not change validity or kill the live guide.
                self.errors += 1
            finally:
                self.pending.task_done()

    def close(self, timeout=.5):
        self.stop.set()
        self.worker.join(timeout=timeout)
        return not self.worker.is_alive()
