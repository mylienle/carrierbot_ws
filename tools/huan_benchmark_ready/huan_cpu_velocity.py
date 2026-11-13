"""Explicit experimental CPU profile; the upstream estimator performs every check."""


def cpu_sparse_estimator(original):
    class CpuSparseEstimator(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            # Four measured images at ~.4 sim seconds do not fit inside .8s.
            # Keep min_samples, max_gap, EMA, outlier and speed limits intact.
            self.window_s = 1.4

        def update(self, tracks, track_history):
            result = super().update(tracks, track_history)
            for track in result:
                track["velocity_odom_filter_profile"] = "cpu_sparse_14_experimental"
                track["velocity_odom_window_s"] = self.window_s
            return result

    return CpuSparseEstimator
