"""Explain visual readiness and latch only an explicit, short start request."""
import math


def packet_status(packet,generation,now):
    result=dict(valid=False,reason='waiting for paired visual measurements',
                front_age_s=None,rear_age_s=None,processing_ms=None,rear_observed_x_m=None)
    if not isinstance(packet,dict) or packet.get('kind')!='packet':return result
    processing=packet.get('processing_ms')
    if isinstance(processing,(int,float)) and math.isfinite(processing):result['processing_ms']=processing
    if packet.get('capture_generation')!=generation:
        result['reason']='capture generation mismatch';return result
    try:
        streams=[packet[name] for name in ('front','rear')]
        times=[float(s['capture_timestamp']) for s in streams]
        if not all(math.isfinite(t) for t in times):raise ValueError('nonfinite capture time')
        for name,t in zip(('front','rear'),times):result[name+'_age_s']=now-t
        xs=[float(p[0]) for p in streams[1].get('points_b',[])]
        if xs and all(math.isfinite(x) for x in xs):result['rear_observed_x_m']=[min(xs),max(xs)]
        independent=packet.get('stream_mode')=='independent_latest'
        if independent and (packet.get('front_path_estimation') is not True
                or type(packet.get('publication_sequence')) is not int):
            raise ValueError('invalid independent stream publication')
        if not independent and (streams[0]['sequence']!=streams[1]['sequence'] or abs(times[0]-times[1])>.04):
            raise ValueError('front/rear pair sequence or receipt skew mismatch')
        for name,t in zip(('front','rear'),times):
            if not -.002<=now-t<=1.:
                raise ValueError('%s capture stale or future (age=%.3f s, limit=1.000 s)'%(name,now-t))
        if not packet.get('valid') or not all(s.get('valid') is True for s in streams):
            raise ValueError('; '.join(name+': '+str(s.get('reason','invalid'))
                for name,s in zip(('front','rear'),streams)))
    except (KeyError,TypeError,ValueError,IndexError) as error:
        result['reason']=str(error);return result
    result.update(valid=True,reason='fresh paired measurements; geometry check required')
    return result


class VisionStartRequest:
    """Wait up to five seconds in ready, without moving until geometry is valid.

    X/B, a second R1+Y, timeout or leaving ready cancels it. A later valid
    frame cannot restart a cancelled request or restart a stopped robot.
    """
    def __init__(self,timeout_s=5.):
        self.timeout_s=timeout_s;self.deadline=None;self.message=None

    @property
    def pending(self):return self.deadline is not None

    def update(self,event,mode,now,usable):
        self.message=None
        if mode!='ready':self.deadline=None;return event
        if event=='track':
            if self.pending:
                self.deadline=None;self.message='visual start cancelled by R1+Y';return None
            self.deadline=now+self.timeout_s
            self.message='visual start requested; waiting up to %.0f s (X/B or R1+Y cancels)'%self.timeout_s
        elif event is not None:
            if self.pending:self.message='visual start cancelled by '+event
            self.deadline=None;return event
        if not self.pending:return event
        if now>=self.deadline:
            self.deadline=None;self.message='visual start timed out; inspect vision reason and press R1+Y again'
            return None
        if usable:
            self.deadline=None;self.message='fresh paired geometry accepted; starting visual tracking'
            return 'track'
        return None


class StableVisualGeometry:
    """Start on distinct consistent measurements; repeated old packets do not count.

    This only gates entry. It never holds a rejected tracking frame as fresh or
    delays a stop. Existing VisionStartRequest still handles cancel/timeout.
    """
    def __init__(self,frames=3):
        self.frames=frames
        self.reset()

    def reset(self):
        self.last=None;self.count=0;self.target_y=None

    def update(self,packet,status):
        if not status.get('valid') or not status.get('rear_valid'):
            self.reset();return False
        front=packet['front'];identity=(front['capture_generation'],front['sequence'])
        if packet.get('stream_mode')=='independent_latest':
            identity=(*identity,packet['rear']['sequence'])
        y=float(status['front_lookahead_b'][1])
        if identity!=self.last:
            if self.last is not None and (identity[0]!=self.last[0] or identity[1]<self.last[1]
                    or abs(y-self.target_y)>.08):
                self.count=0;self.last=None
            if self.last is None or all(a>b for a,b in zip(identity[1:],self.last[1:])):
                self.count+=1;self.last=identity;self.target_y=y
        return self.count>=self.frames
