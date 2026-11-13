"""CameraInfo metadata adapter and explicit estimator profile selection."""
import json
import ast


def configure_runtime_threads(source, requested):
    """Apply the choice before the detector and pose CPU backends start."""
    if requested not in (1, 2):
        raise ValueError("CPU thread choice must be 1 or 2")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == "set_num_threads"
             and isinstance(node.func.value, ast.Name) and node.func.value.id == "torch"]
    direct = [node.value for node in tree.body if isinstance(node, ast.Expr)]
    if (len(calls) != 1 or calls[0] not in direct or len(calls[0].args) != 1
            or not isinstance(calls[0].args[0], ast.Constant) or calls[0].args[0].value != 2
            or calls[0].keywords):
        raise ValueError("CPU source differs: expected one reviewed top-level torch thread setter")
    call = calls[0]
    lines = source.splitlines(keepends=True)
    lines[call.lineno-1:call.end_lineno] = [
        "from ultralytics.utils import torch_utils as _trial_torch_utils\n",
        # Ultralytics select_device() reapplies its CPU thread count on setup.
        "_trial_torch_utils.NUM_THREADS = %d\n" % requested,
        "torch.set_num_threads(%d)\n" % requested,
        "print('[CPU RUNTIME] actual torch_threads=%d interop=%d' % "
        "(torch.get_num_threads(), torch.get_num_interop_threads()), flush=True)\n"]
    result = "".join(lines)
    compile(result, "main_fusion_roi_cpu.py", "exec")
    return result


class MetadataPublisher:
    def __init__(self, owner, publisher):
        self.owner, self.publisher = owner, publisher

    def publish(self, message):
        packet = json.loads(message.data)
        info = self.owner.info
        packet["camera_model"] = dict(
            self.owner.get_intrinsics(), source="camera_info", frame_id=info.header.frame_id,
            width=int(info.width), height=int(info.height), distortion_model=info.distortion_model,
            distortion=list(info.d))
        height, width = self.owner.current[0].shape[:2]
        packet["vision_quality"] = dict(image_width=int(width), image_height=int(height),
                                       adapter="camera_info_metadata_only_v1")
        runtime = getattr(self.owner, "get_cpu_runtime", None)
        if callable(runtime):
            packet["runtime_cpu"] = runtime()
        message.data = json.dumps(packet, allow_nan=False)
        self.publisher.publish(message)


def install(velocity_profile="original"):
    import src.profile_capture as capture
    original = capture.ProfileCapture

    class AnnotatedCapture(original):
        def __init__(self, settings):
            super().__init__(settings)
            self.tracks_pub = MetadataPublisher(self, self.tracks_pub)

        def get_cpu_runtime(self):
            import torch
            return dict(torch_threads_actual=int(torch.get_num_threads()),
                        torch_interop_threads_actual=int(torch.get_num_interop_threads()),
                        velocity_profile_requested=velocity_profile)

    capture.ProfileCapture = AnnotatedCapture
    if velocity_profile == "huan-responsive":
        # Opt-in Jetson filter profile; CPU's lower frame rate may not supply
        # four valid samples inside .6s. Do not select it silently on VMware.
        from huan_fusion_vendor.responsive_filter import ResponsiveEstimator
        import src.odom_velocity_estimator as estimator
        estimator.OdomVelocityEstimator = ResponsiveEstimator
    elif velocity_profile == "cpu-sparse":
        from huan_cpu_velocity import cpu_sparse_estimator
        import src.odom_velocity_estimator as estimator
        estimator.OdomVelocityEstimator = cpu_sparse_estimator(estimator.OdomVelocityEstimator)
    elif velocity_profile != "original":
        raise ValueError("Unknown velocity filter profile")
