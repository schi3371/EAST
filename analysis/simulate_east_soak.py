"""Offline-only repeated lifecycle/transport/recording soak. No hardware imports."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from east_core import load_tester_config
from east_execution import TestEngine, TransportOwner, FeedbackMonitor, RetainedConnectionRecovery, Lifecycle
from east_odrive import ODriveAdapter
from east_reference import AtomicStateStore, ReferenceManager, MotionCoordinator, MotionState
from east_runtime import CSVRecorder, CheckpointWriter

class SimController:
    def __init__(self,axis):
        self.axis=axis
        self.config=SimpleNamespace(absolute_setpoints=False,circular_setpoints=False)
        self._input=0.0
    @property
    def input_pos(self):return self._input
    @input_pos.setter
    def input_pos(self,value):self._input=value;self.axis.pos_vel_mapper.pos_rel=value
class SimAxis:
    def __init__(self):
        self.pos_vel_mapper=SimpleNamespace(pos_rel=0.0,vel=0.0,config=SimpleNamespace(scale=1.0))
        self.current_state=1;self.active_errors=0;self.disarm_reason=0
        self.controller=SimController(self);self.trap_traj=SimpleNamespace(config=SimpleNamespace())
        self.requests=0
    @property
    def requested_state(self):return self.current_state
    @requested_state.setter
    def requested_state(self,value):self.current_state=value;self.requests+=1
class SimDevice:
    def __init__(self):self.axis0=SimAxis();self.serial_number='SIMULATED-NO-HARDWARE'
    @property
    def system_stats(self):return SimpleNamespace(uptime=time.monotonic()*1000)


def run(iterations=100):
    config=load_tester_config()
    coordinator=MotionCoordinator();errors=[]
    engine=TestEngine();owner=TransportOwner(1,lambda:not coordinator.stopping and not coordinator.shutting_down)
    device=SimDevice();adapter=ODriveAdapter(device,0,1,8,transport=owner)
    stop=threading.Event();latest=[None];expect=[None,None]
    monitor=None;monitor_thread=None;writer=None
    maximum_transport_queue=maximum_record_queue=0
    completed=cancelled=recorded=0
    with tempfile.TemporaryDirectory(prefix='east-offline-soak-') as folder:
        manager=ReferenceManager(config,AtomicStateStore(Path(folder)/'state'))
        try:
            def setup():
                initial=adapter.snapshot();latest[0]=initial
                manager.establish_at_physical_neutral(initial,adapter.fingerprint(),'SIMULATED','SIM-FIXTURE',True)
            engine.submit('simulated setup',setup).result(3)
            writer=CheckpointWriter(manager,lambda exc:errors.append(str(exc)));writer.start()
            def start_monitor():
                nonlocal monitor,monitor_thread,stop
                stop=threading.Event()
                monitor=FeedbackMonitor(lambda:adapter,config,stop,latest[0],lambda:tuple(expect),
                    lambda:coordinator.owner,lambda s:latest.__setitem__(0,s),lambda s:None,
                    lambda:None,lambda *args:None,lambda:None,errors.append)
                monitor_thread=threading.Thread(target=monitor.run,name='soak-feedback',daemon=True);monitor_thread.start()
            start_monitor()
            for number in range(iterations):
                def operation():
                    nonlocal completed,cancelled,recorded,maximum_transport_queue,maximum_record_queue
                    token=coordinator.acquire('SIMULATED-run')
                    recorder=CSVRecorder(Path(folder)/f'SIMULATED-{number}.csv',2,lambda exc:errors.append(str(exc)),max_pending_rows=16)
                    recorder.start()
                    try:
                        snapshot=adapter.snapshot()
                        writer.write_and_wait(MotionState.ARMING,snapshot,extra={'simulation':True},timeout_s=1)
                        adapter.request_closed_loop_holding_current(cancelled=lambda:coordinator.stopping)
                        expect[:]=[8,0]
                        engine.transition(Lifecycle.MOVING)
                        for point in (.5,-.5,0):
                            coordinator.assert_active(token)
                            adapter.command_position(point,cancelled=lambda:coordinator.stopping)
                            measured=adapter.snapshot(include_uptime=False)
                            recorder.append((time.monotonic(),measured.position_turns,'SIMULATED'))
                            maximum_transport_queue=max(maximum_transport_queue,owner.pending.qsize())
                            maximum_record_queue=max(maximum_record_queue,recorder.pending.qsize())
                        if number%5==0:
                            coordinator.request_stop();owner.cancel_motion();cancelled+=1
                        else:completed+=1
                        expect[:]=[None,None]
                        idle=adapter.request_idle();assert idle.confirmed
                        coordinator.confirm_motor_idle()
                        recorded+=recorder.finish(1)
                        writer.write_and_wait(MotionState.IDLE,adapter.snapshot(),timeout_s=1)
                    finally:
                        expect[:]=[None,None]
                        if not recorder.done.is_set():recorder.finish(1)
                        coordinator.release(token)
                    assert coordinator.owner is None
                    assert not recorder.thread.is_alive()
                engine.submit('simulated test',operation).result(4)
                if number%10==9:
                    stop.set();monitor_thread.join(1);assert not monitor_thread.is_alive()
                    evidence=engine.submit('simulated recovery',lambda:RetainedConnectionRecovery().run(adapter,lambda:None,config)).result(2)
                    assert not evidence.continuity_proven
                    latest[0]=evidence.feedback
                    start_monitor()
                assert owner.pending.qsize()<=64
                # Drain plain result events: application does this every GUI tick.
                while not engine.events.empty():engine.events.get_nowait()
            assert not errors,errors
        finally:
            stop.set()
            if monitor_thread:monitor_thread.join(1)
            if writer:assert writer.stop(2)
            assert engine.close(2)
            assert owner.close(2)
    remaining=[t.name for t in threading.enumerate() if t.name.startswith(('east-','soak-'))]
    assert not remaining,remaining
    return {'mode':'SIMULATED / NO HARDWARE','iterations':iterations,'completed':completed,
            'operator_stop_sequences':cancelled,'rows_persisted':recorded,'errors':errors,
            'maximum_transport_pending':maximum_transport_queue,'maximum_recording_pending':maximum_record_queue,
            'remaining_workers':remaining,'coordinate_continuity_automatically_restored':False,
            'physical_behaviour':'NOT TESTED'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations',type=int,default=100)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    if not 1<=args.iterations<=10000:parser.error('iterations must be 1–10000')
    result=run(args.iterations);text=json.dumps(result,indent=2)
    if args.output:args.output.write_text(text+'\n',encoding='utf-8')
    print(text)
if __name__=='__main__':main()
