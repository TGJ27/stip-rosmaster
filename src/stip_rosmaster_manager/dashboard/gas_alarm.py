"""MQ-6 relative ADC alarm policy, independent of browser polling.

NOTE: this is percentage change from a CLEAN-AIR ADC baseline, not gas
concentration, percentage LEL, or a certified combustible-gas safety device.
"""
import json
import math
import os
import threading
import time
from pathlib import Path


class GasAlarm:
    def __init__(self, filename):
        self.filename = Path(filename)
        self.lock = threading.RLock()
        self.threshold = 20.0
        self.hysteresis = 3.0
        self.active = False
        self.status = 'WAITING_FOR_SENSOR'
        self.value = None
        self.high_samples = 0
        self.low_samples = 0
        self.last_sample = None
        self.last_event = None
        try:
            cfg = json.loads(self.filename.read_text())
            self.threshold = self._validate(cfg['threshold_percent'])
        except (OSError, ValueError, KeyError, TypeError):
            pass  # Missing or invalid config uses a documented safe default.

    @staticmethod
    def _validate(value):
        if isinstance(value, bool):
            raise ValueError('Threshold must be numeric')
        number = float(value)
        if not math.isfinite(number) or not 1.0 <= number <= 300.0:
            raise ValueError('MQ-6 alarm threshold must be 1–300% ADC change')
        return round(number, 1)

    def save(self, value):
        value = self._validate(value)
        with self.lock:
            self.filename.parent.mkdir(parents=True, exist_ok=True)
            temp = self.filename.with_suffix(self.filename.suffix + '.tmp')
            try:
                temp.write_text(json.dumps({'threshold_percent': value,
                                            'unit': 'relative_adc_percent'}, indent=2)+'\n')
                os.replace(str(temp), str(self.filename))
            finally:
                if temp.exists():temp.unlink()
            self.threshold=value
            self.high_samples=0
            self.low_samples=0
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {'threshold_percent': self.threshold, 'hysteresis_percent': self.hysteresis,
                    'active': self.active, 'status': self.status,
                    'change_percent': self.value, 'last_event': self.last_event,
                    'unit': 'relative_adc_percent'}

    def evaluate(self, ros, now=None):
        """Observe each *new* valid sample once; returns True/False on an edge.

        Missing or stale data never means safe: an asserted alarm stays latched.
        Two successive actual samples avoid single-frame chatter, with a 3-point
        clearance hysteresis below the configured threshold.
        """
        now = time.monotonic() if now is None else now
        with self.lock:
            sample=ros.get('gas_sample_mono')
            health=bool(ros.get('connected') and (ros.get('health') or {}).get('gas'))
            try:
                sample=float(sample)
                value=float(ros.get('gas_change_percent'))
                baseline=float(ros.get('gas_baseline'))
                valid=(health and math.isfinite(sample) and math.isfinite(value)
                       and baseline>0 and 0<=now-sample<=5.0)
            except (ValueError, TypeError):
                valid=False
            if not valid:
                self.status='SENSOR_UNAVAILABLE' if not self.active else 'ALARM_SENSOR_UNAVAILABLE'
                self.value=None
                self.high_samples=0
                self.low_samples=0
                return None
            self.value=round(value,1)
            if self.last_sample == sample:
                self.status='ALARM' if self.active else 'NORMAL'
                return None
            self.last_sample=sample
            if not self.active:
                self.high_samples=self.high_samples+1 if value>self.threshold else 0
                self.status='NORMAL'
                if self.high_samples>=2:
                    self.active=True
                    self.status='ALARM'
                    self.high_samples=0
                    self.last_event=time.strftime('%Y-%m-%d %H:%M:%S')
                    return True
            else:
                self.low_samples=self.low_samples+1 if value<=self.threshold-self.hysteresis else 0
                self.status='ALARM'
                if self.low_samples>=2:
                    self.active=False
                    self.status='NORMAL'
                    self.low_samples=0
                    self.last_event=time.strftime('%Y-%m-%d %H:%M:%S')
                    return False
            return None
