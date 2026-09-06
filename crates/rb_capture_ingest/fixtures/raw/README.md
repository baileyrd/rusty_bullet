# Raw clips (second capture session)

The six whole clips `RB-PHYSICS-001-FR-085` worked from, as recorded by
the BakkesMod plugin (`RB-VERIFY-002-FR-001`) on 2026-09-06, untrimmed.
No test reads them; the trimmed excerpts in the parent directory are the
fixtures. They are kept so the open findings (`F`: the floor-to-wall
curve's speed loss in `walldrive04`; `K`: the ball's goal entry in
`hittickjump01b`; the dodge residuals in `onewheellanding06`) and any
later mechanism can be traced against the full recordings with
`rb-verify --self <clip>` or a reseeded per-tick trace, without a new
session.

| Clip | Frames | What it holds |
|---|---|---|
| `groundjumpthrottle03.jsonl` | 3020 | throttle drive, tapped and held jumps, a landing, a drive up the back wall beside the goal, a brake and a handbrake turn |
| `walldrive04.jsonl` | 4626 | steered driving, a boost run at the cap into the `+X` wall curve, a reverse descent and the `1411` plateau, a second wall climb, a wall jump and a backflip (recorded with no `jump` press — finding I) |
| `curverun05.jsonl` | 5677 | a lap riding the side-wall fillets and all four corner arches with boost and handbrake taps |
| `onewheellanding06.jsonl` | 5350 | many short hops with air roll, diagonal dodges, one-wheel landings |
| `hittickjump01.jsonl` | 5038 | boost, jump at the cap, an airborne hit, a second press with an all-zero stick (finding I); analog axes zero throughout |
| `hittickjump01b.jsonl` | 3131 | boost, jump at the cap, the airborne hit `airborne-hit` is cut from, the ball's goal entry (finding K); analog axes zero throughout |

`jumpbeforehit02.jsonl` was uploaded alongside these and is byte-identical
to `hittickjump01.jsonl`; it is not kept.
