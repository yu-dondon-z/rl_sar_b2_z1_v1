"""Best-effort GUI diagnostics, isolated from camera acquisition and control.

Only reduced copies of already selected images cross the bounded queue. The
viewer never opens a camera, runs inference, or publishes a control command.
"""
import os
from pathlib import Path
import queue
import signal
import sys
import time
import cv2
import numpy as np

FPS=5.
SIZE=(640,360)
TITLES=('VD2 front RGB','VD2 rear D435i RGB-D')


def display_available(environment=None):
    environment=os.environ if environment is None else environment
    # The deployed OpenCV uses GTK/X11. Do not invent DISPLAY=:0: an SSH
    # session or the login greeter does not grant access to a user's desktop.
    return bool(environment.get('DISPLAY'))


def put_latest(output,item):
    """Diagnostic queues may drop frames, but must never wait for the GUI."""
    try:output.put_nowait(item);return True
    except queue.Full:return False


def small_frame(sample,result=None):
    image=sample['image'];height,width=image.shape[:2]
    scale=min(SIZE[0]/width,SIZE[1]/height)
    size=(int(round(width*scale)),int(round(height*scale)))
    left=(SIZE[0]-size[0])//2;top=(SIZE[1]-size[1])//2
    canvas=np.zeros((SIZE[1],SIZE[0],3),np.uint8)
    canvas[top:top+size[1],left:left+size[0]]=cv2.resize(image,size,interpolation=cv2.INTER_AREA)
    # Yellow: neural mask; cyan: pixels surviving geometry/color/rim filters.
    # These are diagnostic pixels, never a claim of valid metric tracking.
    for key,color in (('raw_mask',(0,220,255)),('filtered_mask',(255,220,0))):
        mask=sample.get(key)
        if mask is not None:
            reduced=cv2.resize(mask,size,interpolation=cv2.INTER_NEAREST)>0
            region=canvas[top:top+size[1],left:left+size[0]]
            region[reduced]=(.5*region[reduced]+.5*np.array(color)).astype(np.uint8)
    result=result or {}
    uv=np.asarray(result.get('waypoints_uv',[]),dtype=float).reshape(-1,2)
    uv=uv[np.isfinite(uv).all(axis=1)]
    # Coordinates belong to this exact processed image, never a newer capture.
    uv=uv*np.array([size[0]/width,size[1]/height])+[left,top]
    return dict(image=canvas,waypoints_uv=uv,received=sample['received'],
                number=sample['number'],valid=result.get('valid'),
                reason=result.get('reason','camera preview; detector is not running'))


class PreviewPublisher:
    def __init__(self,output):
        self.output=output;self.last=-float('inf');self.failed=False
        # If a closed/crashed viewer stops reading, its pipe can retain a
        # frame. Diagnostic feeder flushing must not delay camera-worker exit.
        if output is not None and hasattr(output,'cancel_join_thread'):output.cancel_join_thread()

    def due(self,now=None):
        now=time.monotonic() if now is None else now
        return self.output is not None and not self.failed and now-self.last>=1/FPS

    def submit(self,front,rear,packet=None):
        now=time.monotonic()
        if not self.due(now):return False
        self.last=now
        try:
            if self.output.full():return False
            packet=packet or {}
            item=dict(front=small_frame(front,packet.get('front')),
                      rear=small_frame(rear,packet.get('rear')),
                      processing_ms=packet.get('processing_ms'))
            return put_latest(self.output,item)
        except Exception as error:
            self.failed=True
            print('[display] image preview disabled: '+str(error)+'; perception continues.',flush=True)
            return False


def text_line(panel,text,y,color=(230,230,230)):
    # OpenCV's built-in font does not render Chinese; preserve full reasons in
    # the normal terminal/logs and use the source's English diagnostic text.
    text=str(text).encode('ascii',errors='replace').decode()
    cv2.putText(panel,text[:84],(8,y),cv2.FONT_HERSHEY_SIMPLEX,.43,color,1,cv2.LINE_AA)


def render_panel(name,frame,packet,status,now):
    header=190
    panel=np.zeros((SIZE[1]+header,SIZE[0],3),np.uint8)
    frame=frame or {};age=now-frame['received'] if 'received' in frame else None
    stale=age is None or not -.002<=age<=1.
    label='WAIT' if frame.get('valid') is None else 'VALID' if frame['valid'] else 'INVALID'
    if age is not None and age<-.002:label+=' / CLOCK MISMATCH'
    elif stale:label+=' / STALE'
    color=(70,200,70) if label=='VALID' else (0,190,240) if label=='WAIT' else (70,70,255)
    text_line(panel,'%s | frame %s | detection %s'%(name,frame.get('number','--'),label),20,color)
    text_line(panel,'Host receipt age: %s | paired processing: %s ms'%(
        '--' if age is None else '%.3f s'%age,
        '--' if packet.get('processing_ms') is None else '%.1f'%packet['processing_ms']),42,color)
    control_age=now-status.get('monotonic',now)
    text_line(panel,'Control: %s | vx command %.3f m/s | status age %.2f s'%(
        status.get('state','waiting'),status.get('vx_cmd',0.),control_age),64)
    reason=str(status.get('reason','waiting for control status'))
    text_line(panel,'Control reason: '+reason[:65],86)
    text_line(panel,reason[65:145],105)
    reason=str(frame.get('reason','waiting for camera pair'))
    text_line(panel,'Vision reason: '+reason[:65],127,color)
    text_line(panel,reason[65:145],146,color)
    text_line(panel,'Yellow: neural | Cyan: filtered | line: validated | Q/Esc: hide',177)
    if 'image' in frame:
        panel[header:]=frame['image']
        uv=frame['waypoints_uv']
        if len(uv)>1:
            pixels=np.rint(uv+[0,header]).clip([-10000,-10000],[10000,10000]).astype(np.int32)
            cv2.polylines(panel,[pixels],False,color,2,cv2.LINE_AA)
    return panel


def viewer_worker(frames,controls,closing,run_dir):
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    cv2.setNumThreads(1)
    # HighGUI may abort at native-library level on a broken desktop. Only this
    # optional process is lost; camera and actuator IO live elsewhere.
    try:
        for index,title in enumerate(TITLES):
            cv2.namedWindow(title,cv2.WINDOW_AUTOSIZE);cv2.moveWindow(title,index*650,20)
        print('[display] front/rear diagnostic windows opened (up to 5 Hz).',flush=True)
        packet={};status={}
        while not closing.is_set():
            for source in (frames,controls):
                while True:
                    try:item=source.get_nowait()
                    except queue.Empty:break
                    if source is frames:packet=item
                    else:status=item
            now=time.monotonic()
            for title,key in zip(TITLES,('front','rear')):
                cv2.imshow(title,render_panel(title,packet.get(key),packet,status,now))
            if cv2.waitKey(1)&255 in (27,ord('q')):
                print('[display] Q/Esc received in preview window; closing views only.',flush=True)
                break
            # GTK/Wayland backends can return 0 or -1 for WND_PROP_VISIBLE
            # even after successful imshow. Never interpret that unsupported
            # property as a user close. Keep displaying until Q/Esc or shutdown;
            # closing the window decoration may reopen it on the next frame.
            closing.wait(1/FPS)
    except Exception as error:
        message='[display] viewer unavailable: '+str(error)+'; control/perception continue.'
        print(message,flush=True)
        (Path(run_dir)/'display_error.txt').write_text(message+'\n')
    finally:
        try:cv2.destroyAllWindows()
        except cv2.error:pass


class VisualPreview:
    """Owned by the main entry, not by its daemonic camera worker."""
    def __init__(self,context,enabled,run_dir):
        self.frames=self.controls=self.process=self.closing=None
        self.last=-float('inf');self.exit_reported=False
        if not enabled:return
        if not display_available():
            print('[display] DISPLAY is unset; no GUI. Run in the board desktop terminal '
                  'to see windows; SSH alone has no desktop.',flush=True);return
        try:
            self.frames=context.Queue(maxsize=1);self.controls=context.Queue(maxsize=1)
            self.closing=context.Event()
            self.process=context.Process(target=viewer_worker,args=(self.frames,self.controls,self.closing,str(run_dir)),
                                         name='vd2_visual_preview',daemon=True)
            self.process.start()
        except Exception as error:
            print('[display] viewer could not start: '+str(error)+'; continuing without GUI.',flush=True)
            self.close();self.frames=None

    def publish_control(self,state,reason,vx_cmd=0.):
        now=time.monotonic()
        if self.process is None or now-self.last<1/FPS:return
        self.last=now
        if not self.process.is_alive():
            if not self.exit_reported:
                print('[display] windows closed or viewer exited (code %s); control/perception continue.'%
                      self.process.exitcode,flush=True);self.exit_reported=True
            return
        put_latest(self.controls,dict(monotonic=now,state=state,reason=reason,vx_cmd=float(vx_cmd)))

    def close(self):
        if self.closing is not None:self.closing.set()
        if self.process is not None and self.process.pid is not None:
            self.process.join(timeout=1)
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=1)
        for source in (self.frames,self.controls):
            if source is not None:source.cancel_join_thread();source.close()
