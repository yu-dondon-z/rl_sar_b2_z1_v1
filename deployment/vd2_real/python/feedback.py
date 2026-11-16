"""Host receipt freshness without rewriting feedback timestamps or contacts."""


def sample_feedback(lib,snapshot_fn,clock):
    # The native IO thread can update state while Python handles camera output.
    # Read the evaluation clock AFTER copying state so a newly received packet
    # cannot appear newer than the time used to evaluate it.
    state=snapshot_fn(lib)
    now=clock()
    b2_age=now-state.b2_time if state.b2_time>0 else None
    z1_age=now-state.z1_time if state.z1_time>0 else None
    report=dict(fresh=bool(b2_age is not None and z1_age is not None and
                          0<=b2_age<.10 and 0<=z1_age<.10),
        b2_age_s=b2_age,z1_age_s=z1_age,b2_tick=int(state.tick),
        z1_mode=int(state.z1_mode[0]),native_fault=int(state.fault),
        feedback_freshness_scope='B2_DDS_and_Z1_controller_reply',
        z1_controller_reply_age_s=z1_age,z1_motor_sample_age_s=None,
        z1_motor_freshness_known=False)
    return state,now,report


class FeedbackWait:
    """Before takeover, time a continuous gap; after takeover, fail immediately.

    Startup timeout and live-control freshness are separate. Seeing fresh state
    resets only the startup gap timer; it never changes a timestamp or age.
    """
    def __init__(self,timeout=8.):
        self.timeout=timeout;self.missing_since=None

    def update(self,report,now,acquired=False):
        if report['fresh']:
            self.missing_since=None
            return None
        if self.missing_since is None:self.missing_since=now
        if acquired:return 'B2/Z1 feedback expired after takeover'
        if now-self.missing_since>=self.timeout:
            return 'no fresh B2 and Z1 feedback for %.1f continuous seconds'%self.timeout
        return None

    def gap(self,now):
        return 0. if self.missing_since is None else now-self.missing_since


class Z1MotorResponse:
    """Diagnose repeated motor vectors, without claiming a motor sample clock.

    Stationary quantization can repeat. Flag only an exactly unchanged q/dq/tau
    vector still reporting nonzero speed for >=0.2 s. The caller latches policy
    inhibition on this evidence; this does not invent a motor sample timestamp
    or claim that other controller replies prove motor feedback is fresh.
    """
    def __init__(self):
        self.previous=None;self.changed_at=None

    def observe(self,state,now):
        indices=(8,13,14,15,16,17)
        vector=tuple(getattr(state,name)[i] for name in ('q','dq','tau') for i in indices)
        if self.previous!=vector:
            self.previous=vector;self.changed_at=now
        unchanged=now-self.changed_at
        return dict(z1_motor_vector_unchanged_s=unchanged,
                    z1_motor_vector_repeated_while_moving=bool(
                        unchanged>=.2 and max(abs(state.dq[i]) for i in indices)>.05))
