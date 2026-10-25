"""Heading/extrinsic consistency from static-track residuals vs yaw rate (perception-v2 §5.4).

A heading that lags truth by tau sweeps every static track tangentially by -tau * r, so
the residual's regression on yaw rate recovers the lag. A constant bias is unobservable.
"""

from collections import defaultdict, deque

import numpy as np

OK, WARN, ERROR = 0, 1, 2   # diagnostic_msgs/DiagnosticStatus levels


class HeadingCheck:
    """Sliding window of (t, a, r, track_id); result() never alters tracking."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.samples = deque()

    def add(self, t, a, r, track_id):
        self.samples.append((t, a, r, track_id))

    def result(self, now):
        cfg = self.cfg
        while self.samples and self.samples[0][0] < now - cfg['window_s']:
            self.samples.popleft()
        per_track = defaultdict(list)
        for _, a, r, ident in self.samples:
            per_track[ident].append((a, r))
        out = {'level': OK, 'lag_s': 0.0, 'rho': 0.0, 'agree': 0.0, 'n': len(self.samples),
               'tracks': sum(len(v) >= cfg['min_samples_track'] for v in per_track.values()),
               'yaw_rate_std': 0.0, 'excited': False, 'message': 'insufficient excitation'}
        if not self.samples:
            return out
        a = np.array([s[1] for s in self.samples])
        r = np.array([s[2] for s in self.samples])
        out['yaw_rate_std'] = float(r.std())
        if out['n'] < cfg['min_samples'] or out['tracks'] < cfg['min_tracks'] \
                or out['yaw_rate_std'] < cfg['min_yaw_rate_std']:
            return out
        cov = float(np.mean((a - a.mean()) * (r - r.mean())))
        slope = cov / float(r.var())
        out['lag_s'] = -slope
        out['rho'] = cov / max(float(a.std() * r.std()), 1e-12)
        signs = []
        for rows in per_track.values():
            if len(rows) < cfg['min_samples_track']:
                continue
            ta, tr = np.array(rows).T
            if tr.var() > 0:
                signs.append(np.sign(np.mean((ta - ta.mean()) * (tr - tr.mean()))) ==
                             np.sign(slope))
        out['agree'] = float(np.mean(signs)) if signs else 0.0
        out['excited'] = True
        correlated = abs(out['rho']) >= cfg['rho_warn'] and out['agree'] >= cfg['agree_min']
        if correlated and abs(out['lag_s']) >= cfg['lag_error_s']:
            out['level'] = ERROR
        elif correlated and abs(out['lag_s']) >= cfg['lag_warn_s']:
            out['level'] = WARN
        out['message'] = ('heading or camera extrinsic lags during turns'
                          if out['level'] else 'ok')
        return out
