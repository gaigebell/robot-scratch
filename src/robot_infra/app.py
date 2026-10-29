"""Fixed M1 scenario orchestration through Session; no direct device access."""

from dataclasses import replace

from .contracts import canonical
from .fake import FakeTeleoperator
from .graph import validate_graph
from .recording import Recorder, replay, summary
from .runtime import Session


def run_demo(config, output, *, steps=12, period_ns=None):
    if type(steps) is not int or not 1 <= steps <= 10000:
        raise ValueError("steps must be between 1 and 10000")
    config = validate_graph(config)["config"]
    with Recorder(output) as recorder:
        session = Session(config, sink=recorder.write)
        session.start()
        period_ns = session.config["timing"]["period_ns"] if period_ns is None else period_ns
        if type(period_ns) is not int or period_ns < 1:
            raise ValueError("period must be a positive integer")
        lease = session.acquire("teleop")
        teleop = FakeTeleoperator()
        for step in range(steps):
            lease = session.renew(lease)
            session.observe()
            session.submit(session.intent("teleop", teleop.sample(step), lease))
            session.clock.advance(period_ns)
        session.release(lease)
        lease = session.acquire("policy")
        request = session.request_policy(delay_ns=period_ns)
        session.clock.advance(period_ns)
        session.complete_policy(request)
        session.tick()
        # Demonstrate queue cancellation and a truly delayed pre-takeover result.
        late = session.request_policy(delay_ns=period_ns)
        lease = session.acquire("teleop", takeover=True)
        session.clock.advance(period_ns)
        session.complete_policy(late)
        session.observe()
        session.submit(session.intent("teleop", (0.1, 0.1), lease))
        session.submit(session.intent("teleop", (99.0, 0.0), lease))
        expired = session.intent("teleop", (0.0, 0.0), lease)
        session.submit(replace(expired, expires_ns=session.clock.now_ns))
        session.submit(session.intent("policy", (0.0, 0.0)))
        session.stop()
        session.finalize()
    events = replay(output)
    if canonical(events) != canonical(session.events):
        raise ValueError("recorded/replayed timeline differs from live timeline")
    return summary(events)
