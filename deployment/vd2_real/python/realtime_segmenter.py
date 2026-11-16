"""Unchanged RHANet inputs/weights/threshold; efficient CPU tensor layout."""
import torch
from crack_inference import CrackSegmenter


class RealtimeSegmenter(CrackSegmenter):
    def predict_bgr(self,image):
        result=super().predict_bgr(image)
        self.last_diagnostic_mask=result.mask.copy()
        return result

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.channels_last=self.device.type=='cpu'
        if self.channels_last:self.model.to(memory_format=torch.channels_last)

    def _to_tensor(self,rgb):
        tensor=super()._to_tensor(rgb)
        return tensor.contiguous(memory_format=torch.channels_last) if self.channels_last else tensor

