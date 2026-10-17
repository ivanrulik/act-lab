# Bounded ACT continuation to 10,000 updates

Date: 2026-10-16. The **18/20 diagnostic gate failed**. Full
[metrics](act-budget-10000-metrics.json) preserve every case, sampled predictions,
exact counters, configuration, checkpoint hashes and explicit resume provenance.
No final-test cohort was consumed.

The official pipeline continued from 5,000 to 10,000 updates on unchanged data,
model, seed, batch, optimizer, rates, images and safety. The original loader stayed
active throughout. The five-thousand-step checkpoint remained unchanged. The
additional 5,000 updates took about 44 minutes; final displayed L1 was 0.048.
Source revision for the resume is in its sidecar; the original manifest's initial
Git field is not represented as the resume-code identity.

| Diagnostic seeds 21000–21019, coefficient 0.01 | 5,000 steps | 10,000 steps |
| --- | ---: | ---: |
| Task success / grasp events | 3/20 | **8/20** |
| Collision-stop commands | 827 | **1,469** |
| Drops | 0 | 0 |
| Median sampled XYZ error, train | 18.46 mm | **6.06 mm** |
| Median sampled XYZ error, reserved | 18.54 mm | **8.09 mm** |
| p95 decision wall time | 8.75 ms | 10.83 ms |

Two old successes were retained (21005, 21007), six gained (21004, 21009, 21011,
21012, 21013, 21018), and one lost (21000). Model selection is not uniformly
monotonic across seeds. Wilson 95% interval is 21.88–61.34%; these are repeatedly
used diagnostic seeds, not independent final qualification. All twelve remaining
cases timed out. Mean successful completion was 10.56 seconds.

The collision count is **rejected commands**, including repeated holds, not
1,469 observed impacts. It spans eight seeds and remains a material problem.
No guard was weakened to obtain more successes. Decision p99 was 17.48 ms and
maximum 317.21 ms; offline timing is not hardware real-time evidence.

A matched-frame comparison of the older checkpoint's heads 0/25 found no
consistent first-head disadvantage; do not infer a need for first-step loss
reweighting from aggregate loss alone. Terminal-frame sampling also affects the
five-frame-per-episode descriptive error summary.

The [next conditional protocol](../plans/act-budget-20000-study.md) requires the
isolated numeric-loading equivalence/throughput gate before another bounded
budget extension. Its learning hypothesis is supported by reduced sampled errors
and observed task gains, while collision outcomes and final qualification remain
unresolved. No hardware use or publication is approved.
