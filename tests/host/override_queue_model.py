"""N111 handoff protocol prototype; deliberately NOT a motion/jerk planner.

One tick executes a short block. Producer steps prepare one immutable row, or
promote one claimed row back into the ordinary bank. Arithmetic costs are input
work units, not estimates of AVR cycles. The smooth speed samples only expose
mixed boundaries; their physical feasibility belongs to the real planner.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Row:
    block: int
    generation: int
    entry: float
    exit: float


@dataclass
class Candidate:
    request: int
    epoch: int
    anchor: int
    start_speed: float
    factor: float
    rows: dict = field(default_factory=dict)
    progress: int = 0
    ready: bool = False
    claimed: bool = False


class QueueModel:
    def __init__(self, capacity=32, lead=16, row_cost=2, ramp=8):
        self.capacity, self.lead, self.row_cost, self.ramp = capacity, lead, row_cost, ramp
        self.queue = list(range(capacity))
        self.next_block = capacity
        self.normal = {i: Row(i, 0, 1., 1.) for i in self.queue}
        self.candidate = None
        self.request = self.epoch = self.generation = 0
        self.factor = self.applied = self.speed = 1.
        self.pending = False
        self.trace = []
        self.events = []
        self.misses = self.claims = self.retries = 0

    def set_factor(self, factor):
        self.request += 1
        self.factor = factor
        self.pending = True
        # A claimed bank remains readable until promoted; a new request cannot
        # overwrite a row still owned by the consumer.
        if self.candidate and not self.candidate.claimed:
            self.candidate = None

    def cancel(self):
        self.epoch += 1
        self.pending = False
        if self.candidate and not self.candidate.claimed:
            self.candidate = None
        self.events.append(('cancel', self.epoch))

    def _row(self, block):
        c = self.candidate
        if c and c.claimed and block in c.rows:
            return c.rows[block]
        return self.normal[block]

    def _start(self):
        # The anchor is a boundary not yet executed; the prototype assumes its
        # incoming speed is feasible. Real planning must also check acceleration.
        if not self.pending or len(self.queue) <= self.lead:
            return
        anchor = self.queue[self.lead]
        self.candidate = Candidate(self.request, self.epoch, anchor,
                                   self._row(anchor).entry, self.factor)
        self.retries += 1

    def producer(self):
        c = self.candidate
        if c and c.claimed:
            # Copying one row is interruptible: consumers still use shadow until
            # its validity bit is cleared. Model that last byte as this event.
            if c.rows:
                block = min(c.rows)
                if block in self.normal:
                    self.normal[block] = c.rows[block]
                del c.rows[block]
                return
            self.candidate = None
            c = None
        if c is None:
            self._start()
            c = self.candidate
        if c is None:
            return
        if c.epoch != self.epoch or c.request != self.request:
            self.candidate = None
            return
        missing = [b for b in self.queue if b >= c.anchor and b not in c.rows]
        if not missing:
            c.ready = True
            return
        c.ready = False
        c.progress += 1
        if c.progress < self.row_cost:
            return
        c.progress = 0
        block = missing[0]
        def velocity(boundary):
            t = min(1., max(0., (boundary-c.anchor)/self.ramp))
            s = t*t*t*(10+t*(-15+6*t))
            return c.start_speed+(c.factor-c.start_speed)*s
        c.rows[block] = Row(block, c.request, velocity(block), velocity(block+1))
        c.ready = all(b in c.rows for b in self.queue if b >= c.anchor)

    def execute(self, refill=True):
        if not self.queue:
            raise AssertionError('unexpected empty queue')
        block = self.queue[0]
        c = self.candidate
        if c and not c.claimed and block >= c.anchor:
            valid = (block == c.anchor and c.ready and c.request == self.request
                     and c.epoch == self.epoch
                     and all(b in c.rows and c.rows[b].block == b
                             for b in self.queue if b >= c.anchor)
                     and c.rows[block].entry == self.speed)
            if valid:
                c.claimed = True
                self.generation, self.applied = c.request, c.factor
                self.pending = False
                self.claims += 1
                self.events.append(('claim', block, self.generation))
            else:
                self.candidate = None
                self.misses += 1
                self.events.append(('miss', block))
                # Retry farther ahead, preserving at least the modeled ramp.
                # Firmware must derive this distance from jerk feasibility.
                self.lead = min(self.capacity-self.ramp, self.lead+4)
        row = self._row(block)
        assert row.block == block  # slot reuse must not revive an old row
        assert row.entry == self.speed, (block, row, self.speed)
        assert row.entry > 0 and row.exit > 0  # no artificial stop
        self.speed = row.exit
        self.trace.append(row)
        self.queue.pop(0)
        del self.normal[block]
        c = self.candidate
        if c and c.claimed:
            c.rows.pop(block, None)
        if refill:
            b = self.next_block
            self.next_block += 1
            last = self._row(self.queue[-1]) if self.queue else row
            # Refill after claim inherits the committed plan. Before claim it
            # stays on the ordinary generation and extends the candidate's work.
            self.normal[b] = Row(b, self.generation, last.exit, self.applied)
            self.queue.append(b)
            if c and not c.claimed:
                c.ready = False
        return row

    def check(self):
        assert all(a.exit == b.entry for a, b in zip(self.trace, self.trace[1:]))
        assert all(r.entry > 0 and r.exit > 0 for r in self.trace)
        assert all(b.block == a.block+1 for a, b in zip(self.trace, self.trace[1:]))
