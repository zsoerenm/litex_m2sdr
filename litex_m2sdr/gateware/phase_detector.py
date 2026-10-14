#
# This file is part of LiteX-M2SDR.
#
# SPDX-License-Identifier: BSD-2-Clause

"""Wire-based sub-sample phase detector for two boards sharing the 38.4MHz reference on CN6.

One AD9361 sample period at 7.68MSPS is exactly 5 periods T of the shared 38.4MHz reference, so
the reference edges alone are ambiguous by 0..4 T. The clock master labels one edge with a marker
(SyncMarkerTx): a single reference period with 25% instead of 50% duty cycle. Every rising edge
stays in place, so the SI5351C/MMCM loops that lock to the reference are not disturbed.

Both boards (PhaseDetector) run identical logic from the reference as seen on their own CN6 pad
(the master's pad input sees its own output):

- pd_mmcm (VCO 32 x 38.4MHz) gives "pd_ref" (0 deg) and "pd_det" (135 deg nominal, fine phase
  shift for calibration). pd_det samples the pad between the marker's and the normal falling edge;
  a low sample is a marker and resets a mod-5 period counter in pd_ref. The counter labels every
  reference period identically on both boards; "r" is high for labels 0 and 1.
- ps_mmcm (same VCO) gives "pd_ps", swept by its fine phase shift in 1/56 VCO period steps:
  1792 steps = one reference period T. pd_ps samples s (the AD9361 frame-rate square wave from
  the RX PHY, rfic domain), r and the CN6 pad. With a free-running mod-5 count of pd_ps cycles,
  the sample with label j at sweep step k is taken at j*T + k*T/1792 (+ a common offset), so one
  sweep maps each signal over 5T = one sample period in 8960 bins. The position of s relative to r
  is the AD9361 frame timing relative to the labelled reference edge; the difference between the
  boards is their sub-sample offset plus a constant (cable, routing).
"""

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.cores.clock import S7MMCM

REF_FREQ    = 38.4e6
VCO_MULT    = 32                  # VCO = 1228.8MHz; fine phase shift step = VCO period / 56.
SWEEP_STEPS = 56 * VCO_MULT       # 1792 steps = one reference period.
NLABELS     = 5                   # Reference periods per sample period (7.68MSPS).
NSAMPLES    = 63                  # Samples per label and sweep step (6-bit counts).

# Marker Generator (Clock Master) ------------------------------------------------------------------

class SyncMarkerTx(LiteXModule):
    """Reference output with an on-demand duty-cycle marker (clock master).

    The CN6 and SI5351C CLKIN ODDRs run from "cn6x2" (2 x 38.4MHz, cascaded from clk10) with data
    ph (high on even cn6x2 cycles): 50% duty, rising edges on the same cn6x2 edges as before.
    A marker clears the second half of one high period (25% duty). Markers only start on every
    tenth cn6x2 cycle, i.e. every fifth reference period, so repeated markers keep the period
    labels of the receivers consistent.
    """
    def __init__(self):
        self.cd_cn6x2 = ClockDomain()
        self.ph       = Signal()   # ODDR data, cn6x2 domain.
        self.cn6_d1   = Signal()
        self.cn6_d2   = Signal()

        self.fire = CSR()
        self.sent = CSRStatus(16, description="Markers sent.")

        # # #

        self.mmcm = mmcm = S7MMCM(speedgrade=-3, fractional=False, name="cn6x2_mmcm")
        self.comb += mmcm.reset.eq(ResetSignal("clk10"))
        mmcm.register_clkin(ClockSignal("clk10"), REF_FREQ)
        mmcm.create_clkout(self.cd_cn6x2, 2*REF_FREQ, margin=0)

        fire = PulseSynchronizer("sys", "cn6x2")
        self.submodules += fire
        self.comb += fire.i.eq(self.fire.re)

        cnt     = Signal(4)
        pending = Signal()
        marker  = Signal()
        sent    = Signal(16)
        self.sync.cn6x2 += [
            cnt.eq(Mux(cnt == 9, 0, cnt + 1)),
            If(fire.o, pending.eq(1)),
            marker.eq(0),
            If((cnt == 9) & pending,
                marker.eq(1),
                pending.eq(0),
                sent.eq(sent + 1),
            ),
        ]
        self.comb += [
            self.ph.eq(~cnt[0]),
            self.cn6_d1.eq(self.ph),
            self.cn6_d2.eq(self.ph & ~marker),
        ]
        self.specials += MultiReg(sent, self.sent.status)

# Phase Detector -----------------------------------------------------------------------------------

def _mmcm(name, clkin, psclk, psen, psincdec, psdone, rst, locked, outs):
    """MMCME2_ADV at VCO = 32 x 38.4MHz, outputs [(clk, phase, fine_ps)] at 38.4MHz via BUFG."""
    fb     = Signal()
    params = dict(
        p_BANDWIDTH       = "OPTIMIZED",
        p_REF_JITTER1     = 0.01,
        p_CLKIN1_PERIOD   = 1e9/REF_FREQ,
        p_CLKFBOUT_MULT_F = float(VCO_MULT),
        p_DIVCLK_DIVIDE   = 1,
        i_CLKIN1          = clkin,
        i_CLKFBIN         = fb,
        o_CLKFBOUT        = fb,
        i_RST             = rst,
        i_PWRDWN          = 0,
        o_LOCKED          = locked,
        i_PSCLK           = psclk,
        i_PSEN            = psen,
        i_PSINCDEC        = psincdec,
        o_PSDONE          = psdone,
    )
    specials = []
    for n, (clk, phase, fine_ps) in enumerate(outs):
        unbuf = Signal()
        params[f"p_CLKOUT{n}_DIVIDE" + ("_F" if n == 0 else "")] = float(VCO_MULT) if n == 0 else VCO_MULT
        params[f"p_CLKOUT{n}_PHASE"]       = float(phase)
        params[f"p_CLKOUT{n}_USE_FINE_PS"] = "TRUE" if fine_ps else "FALSE"
        params[f"o_CLKOUT{n}"]             = unbuf
        specials.append(Instance("BUFG", i_I=unbuf, o_O=clk))
    specials.append(Instance("MMCME2_ADV", name=name, **params))
    return specials

class PhaseDetector(LiteXModule):
    def __init__(self, cn6, rx_count):
        self.cd_pd_ref = ClockDomain(reset_less=True)
        self.cd_pd_det = ClockDomain(reset_less=True)
        self.cd_pd_ps  = ClockDomain(reset_less=True)

        self.control = CSRStorage(fields=[
            CSRField("mmcm_rst", size=1, offset=0, description="Hold both MMCMs in reset (phases back to nominal)."),
            CSRField("start",    size=1, offset=1, pulse=True, description="Start a sweep."),
            CSRField("det_inc",  size=1, offset=2, pulse=True, description="Shift pd_det one step later."),
            CSRField("det_dec",  size=1, offset=3, pulse=True, description="Shift pd_det one step earlier."),
        ])
        self.status = CSRStatus(fields=[
            CSRField("pd_locked", size=1,  offset=0),
            CSRField("ps_locked", size=1,  offset=1),
            CSRField("busy",      size=1,  offset=2),
            CSRField("sweeps",    size=16, offset=16, description="Completed sweeps."),
        ])
        self.markers  = CSRStatus(32, description="[31:16] markers detected, [15:0] markers that changed the labels.")
        self.det_pos  = CSRStatus(16, description="pd_det fine phase shift (signed steps).")
        self.rd_addr  = CSRStorage(11, description="Sweep memory read address (sweep step).")
        self.rd_data  = CSRStatus(96, description="Per label j (6 bits at 6j): [95:64] CN6 pad, [63:32] r, [29:0] s high counts.")

        # # #

        # Clocking.
        # ---------
        cn6_bufg = Signal()
        self.specials += Instance("BUFG", name="pd_bufg", i_I=cn6, o_O=cn6_bufg)

        mmcm_rst  = self.control.fields.mmcm_rst
        pd_locked = Signal()
        ps_locked = Signal()
        det_psen, det_psincdec, det_psdone = Signal(), Signal(), Signal()
        ps_psen,  ps_psincdec,  ps_psdone  = Signal(), Signal(), Signal()
        self.specials += _mmcm("pd_mmcm", cn6_bufg, ClockSignal("sys"),
            det_psen, det_psincdec, det_psdone, mmcm_rst, pd_locked, [
            (self.cd_pd_det.clk, 135.0, True),
            (self.cd_pd_ref.clk,   0.0, False),
        ])
        self.specials += _mmcm("ps_mmcm", cn6_bufg, ClockSignal("sys"),
            ps_psen, ps_psincdec, ps_psdone, mmcm_rst, ps_locked, [
            (self.cd_pd_ps.clk, 0.0, True),
        ])
        self.specials += [
            MultiReg(pd_locked, self.status.fields.pd_locked),
            MultiReg(ps_locked, self.status.fields.ps_locked),
        ]

        # pd_det fine phase (calibration of the marker sampling point).
        det_busy = Signal()
        det_pos  = Signal((16, True))
        self.sync += [
            det_psen.eq(0),
            If(~det_busy & (self.control.fields.det_inc | self.control.fields.det_dec),
                det_psen.eq(1),
                det_psincdec.eq(self.control.fields.det_inc),
                det_busy.eq(1),
                det_pos.eq(det_pos + Mux(self.control.fields.det_inc, 1, -1)),
            ),
            If(det_psdone, det_busy.eq(0)),
            If(mmcm_rst, det_pos.eq(0), det_busy.eq(0)),
        ]
        self.comb += self.det_pos.status.eq(det_pos)

        # Marker detection and period labels.
        # -----------------------------------
        det_sample = Signal(reset=1)
        self.specials += MultiReg(cn6, det_sample, odomain="pd_det", reset=1)  # 1st stage samples.
        det_marker = Signal()
        self.sync.pd_det += det_marker.eq(~det_sample)

        label    = Signal(3)
        marker   = Signal()
        r        = Signal()
        detected = Signal(16)
        relabel  = Signal(16)
        self.sync.pd_ref += [
            marker.eq(det_marker),
            If(marker,
                label.eq(0),
                detected.eq(detected + 1),
                If(label != NLABELS - 1, relabel.eq(relabel + 1)),
            ).Else(
                label.eq(Mux(label == NLABELS - 1, 0, label + 1)),
            ),
            r.eq(label < 2),
        ]
        self.specials += MultiReg(Cat(relabel, detected), self.markers.status)

        # AD9361 frame-rate square wave (rfic domain).
        # --------------------------------------------
        s = Signal()
        self.sync.rfic += s.eq(rx_count[1])

        # Sampling (pd_ps domain).
        # ------------------------
        # MultiReg first stages are the samples (false paths); all three see the same latency.
        s_ps, r_ps, c_ps = Signal(), Signal(), Signal()
        self.specials += [
            MultiReg(s,   s_ps, odomain="pd_ps"),
            MultiReg(r,   r_ps, odomain="pd_ps"),
            MultiReg(cn6, c_ps, odomain="pd_ps"),
        ]
        # Blocks of NSAMPLES x NLABELS cycles: per label, count the high samples of each signal.
        # The first cycle of a block latches the previous block's counts and toggles "toggle".
        lbl    = Signal(3)
        n      = Signal(6)
        acc    = [[Signal(6) for _ in range(NLABELS)] for _ in range(3)]
        res    = Signal(96)
        toggle = Signal()
        self.sync.pd_ps += [
            lbl.eq(Mux(lbl == NLABELS - 1, 0, lbl + 1)),
            If(lbl == NLABELS - 1, n.eq(Mux(n == NSAMPLES - 1, 0, n + 1))),
            If((lbl == 0) & (n == 0),
                res.eq(Cat(*[Cat(*acc[k], C(0, 2)) for k in range(3)])),
                toggle.eq(~toggle),
            ),
        ]
        for k, x in enumerate([s_ps, r_ps, c_ps]):
            for j in range(NLABELS):
                self.sync.pd_ps += If(lbl == j,
                    If(n == 0,
                        acc[k][j].eq(x),
                    ).Else(
                        acc[k][j].eq(acc[k][j] + x),
                    )
                )

        # Sweep (sys domain).
        # -------------------
        # Per step: wait for two block toggles (a complete block at the new phase), store the counts,
        # then shift pd_ps one step later. SWEEP_STEPS steps cover one reference period.
        toggle_sys = Signal()
        res_sys    = Signal(96)
        self.specials += [
            MultiReg(toggle, toggle_sys),
            MultiReg(res,    res_sys),
        ]
        toggle_d = Signal()
        seen     = Signal(2)
        wait     = Signal(4)
        step     = Signal(11)
        sweeps   = Signal(16)
        mem      = Memory(96, SWEEP_STEPS)
        wr_port  = mem.get_port(write_capable=True)
        rd_port  = mem.get_port()
        self.specials += mem, wr_port, rd_port
        self.comb += [
            rd_port.adr.eq(self.rd_addr.storage),
            self.rd_data.status.eq(rd_port.dat_r),
            wr_port.adr.eq(step),
            wr_port.dat_w.eq(res_sys),
            self.status.fields.sweeps.eq(sweeps),
        ]
        self.sync += toggle_d.eq(toggle_sys)

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.control.fields.start & ~mmcm_rst,
                NextValue(step, 0),
                NextValue(seen, 0),
                NextState("WAIT"),
            )
        )
        fsm.act("WAIT",
            self.status.fields.busy.eq(1),
            If(toggle_sys != toggle_d,
                NextValue(seen, seen + 1),
                If(seen == 1,
                    NextValue(wait, 15),  # Let the res MultiRegs settle (res changed with toggle).
                    NextState("SETTLE"),
                )
            )
        )
        fsm.act("SETTLE",
            self.status.fields.busy.eq(1),
            NextValue(wait, wait - 1),
            If(wait == 0, NextState("STORE"))
        )
        fsm.act("STORE",
            self.status.fields.busy.eq(1),
            wr_port.we.eq(1),
            If(step == SWEEP_STEPS - 1,
                NextValue(sweeps, sweeps + 1),
                NextState("IDLE"),
            ).Else(
                ps_psen.eq(1),
                NextState("SHIFT"),
            )
        )
        fsm.act("SHIFT",
            self.status.fields.busy.eq(1),
            If(ps_psdone,
                NextValue(step, step + 1),
                NextValue(seen, 0),
                NextState("WAIT"),
            )
        )
        self.comb += ps_psincdec.eq(1)
