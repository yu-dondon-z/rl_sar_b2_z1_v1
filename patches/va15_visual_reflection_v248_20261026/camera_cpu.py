"""Same RGB network/weights/resolution, CPU memory-layout optimization only."""
def configure_cpu_layout(segmenter):
    import torch

    if segmenter.device.type != 'cpu' or segmenter.input_size != (360, 640):
        raise RuntimeError('Expected the unchanged CPU 360x640 camera contract')
    # RHACrackNet contains depthwise convolutions. Channels-last weights avoid
    # repeated CPU layout conversions. No tracing, quantization, retraining,
    # changed input resolution or changed confidence/identity limits.
    generator = torch.Generator().manual_seed(244)
    sample = torch.rand((1, 3, 360, 640), generator=generator)
    with torch.inference_mode():
        before = segmenter.model(sample).detach().clone()
        segmenter.model.to(memory_format=torch.channels_last)
        after = segmenter.model(sample.contiguous(memory_format=torch.channels_last))
        error = float((before - after).abs().max())
        if (not torch.isfinite(after).all() or
                not torch.allclose(before, after, atol=5.e-5, rtol=1.e-4)):
            raise RuntimeError('CPU memory-layout golden mismatch: ' + str(error))
    return dict(input_size=[360, 640], memory_layout='channels_last',
                golden_max_abs_error=error, torch_version=torch.__version__,
                checkpoint_changed=False, resolution_changed=False)


class PublicationTiming:
    """Measure the actual expiry budget; never grants motion permission."""
    def __init__(self):
        self.last_valid_capture_ns = 0

    def annotate(self, packet):
        capture = packet['capture_monotonic_ns']
        publish = packet['publish_monotonic_ns']
        previous = self.last_valid_capture_ns
        packet['capture_to_publication_ms'] = (publish-capture)/1.e6
        packet['previous_valid_age_at_publication_ms'] = (
            (publish-previous)/1.e6 if previous else None)
        if packet.get('valid'):
            self.last_valid_capture_ns = capture

