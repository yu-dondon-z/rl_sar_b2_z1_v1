"""Small ctypes boundary; loading this module does not initialize transports."""
import ctypes as ct
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parents[1]

class State(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in ('b2_time', 'z1_time', 'command_time')]
    _fields_ += [(name, ct.c_double * n) for name, n in
                 (('q',18),('dq',18),('tau',18),('quaternion',4),('omega',3))]
    _fields_ += [('tick',ct.c_uint32),('b2_flags',ct.c_uint32),
                 ('motor_fault',ct.c_uint32*12),('motor_lost',ct.c_uint32*12)]
    _fields_ += [(name,ct.c_int32*n) for name,n in
                 (('z1_mode',1),('z1_error',6),('z1_connected',6),('foot_force',4))]
    _fields_ += [('keys',ct.c_uint32),('fault',ct.c_uint32)]
    _fields_ += [('z1_sent_time',ct.c_double),('z1_sent_source_time',ct.c_double),
                ('z1_sent_q',ct.c_double*6),('z1_sends',ct.c_uint64),
                ('z1_suppressed',ct.c_uint64),('z1_replies',ct.c_uint64)]

class Command(ct.Structure):
    _fields_ = [('q',ct.c_double*18),('policy_blend',ct.c_double),('mode',ct.c_uint32),
                ('leg_kp',ct.c_double*18),('leg_kd',ct.c_double*18)]

def library():
    # Select bundled CycloneDDS before any ROS installation on the host can
    # satisfy its SONAME with a different build/ABI.
    sdk=ROOT/'thirdparty/unitree_sdk2/thirdparty/lib'/platform.machine()
    ct.CDLL(str(sdk/'libddsc.so'),mode=ct.RTLD_GLOBAL)
    ct.CDLL(str(sdk/'libddscxx.so'),mode=ct.RTLD_GLOBAL)
    lib = ct.CDLL(str(ROOT/'build/lib/libvd2_io.so'))
    lib.vd2_error.restype = ct.c_char_p
    lib.vd2_state_size.restype = ct.c_ulong
    if lib.vd2_state_size()!=ct.sizeof(State):raise RuntimeError('C++/Python state ABI size differs')
    lib.vd2_command_size.restype = ct.c_ulong
    if lib.vd2_command_size()!=ct.sizeof(Command):
        raise RuntimeError('C++/Python command ABI size differs; rebuild with bash build_board.sh')
    lib.vd2_open.argtypes = [ct.c_char_p,ct.c_char_p,ct.c_int,ct.c_int]
    lib.vd2_snapshot.argtypes = [ct.POINTER(State)]
    lib.vd2_command.argtypes = [ct.POINTER(Command)]
    lib.vd2_front_open.argtypes = [ct.c_char_p]
    lib.vd2_front_image.argtypes = [ct.POINTER(ct.c_ubyte),ct.c_int,
                                   ct.POINTER(ct.c_double),ct.POINTER(ct.c_double)]
    return lib

def check(lib, result):
    if result:
        raise RuntimeError(lib.vd2_error().decode(errors='replace'))

def send(lib, q, mode=1, policy_blend=0.,leg_kp=None,leg_kd=None):
    if leg_kp is None or leg_kd is None:
        from control import FIXED_KP,FIXED_KD,POLICY_KP,POLICY_KD
        if leg_kp is None:leg_kp=FIXED_KP+(POLICY_KP-FIXED_KP)*policy_blend
        if leg_kd is None:leg_kd=FIXED_KD+(POLICY_KD-FIXED_KD)*policy_blend
    command = Command((ct.c_double*18)(*q), policy_blend, mode)
    command.leg_kp[:]=leg_kp;command.leg_kd[:]=leg_kd
    check(lib,lib.vd2_command(ct.byref(command)))

def snapshot(lib):
    result=State()
    check(lib,lib.vd2_snapshot(ct.byref(result)))
    return result
