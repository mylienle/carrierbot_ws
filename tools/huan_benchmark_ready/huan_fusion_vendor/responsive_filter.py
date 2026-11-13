"""Opt-in simulation profile selected on slow/moderate recorded observations.

Retains all original time, missing-XYZ, frame, outlier and speed rejection rules.
"""
from src.odom_velocity_estimator import OdomVelocityEstimator as OriginalEstimator


class ResponsiveEstimator(OriginalEstimator):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.window_s=.6
        self.ema_alpha=.65
    def update(self,tracks,track_history):
        result=super().update(tracks,track_history)
        for t in result:t['velocity_odom_filter_profile']='responsive_06'
        return result
