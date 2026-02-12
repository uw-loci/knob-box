```mermaid
stateDiagram-v2
  direction TB

  state "STATE_INTERLOCK\n(BI / INTERLOCK)\n\nOutputs:\nCCS(A0)=OFF\nBeam(A1)=OFF\n3kV(A2)=sw_3kv_enable\n\nNomOp flag: LOW" as INTERLOCK
  state "STATE_NOM_OP\n(NOMINAL OP)\n\nOutputs:\n3kV(A2)=sw_3kv_enable\nCCS(A0)=sw_ccs_allow\nBeam(A1)=sw_arm_beams\n\nNomOp flag: HIGH" as NOMOP
  state "STATE_3KV_TIMER\n(3kV TIMER)\n\nOutputs:\nCCS(A0)=OFF\nBeam(A1)=OFF\n3kV(A2)=OFF (forced)\n\nHold 100 ms then -> INTERLOCK\n\nNomOp flag: LOW" as TIMER

  [*] --> INTERLOCK : setup()

  INTERLOCK --> TIMER : 3kV_I fault
  note on link
    Entry actions:
    - timerEnterMs = millis()
    - enable3kV forced OFF
    - break early
  end note

  INTERLOCK --> NOMOP : reset edge AND all safe AND arm_80kv AND 3kV_en

  NOMOP --> TIMER : 3kV_I or 3kV_V fault
  note on link
    Entry actions:
    - timerEnterMs = millis()
    - CCS OFF, Beam OFF, 3kV OFF
    - break early
  end note

  NOMOP --> INTERLOCK : (comparators != 0) OR !arm_80kv OR !3kV_en
  note on link
    Transition actions:
    - CCS OFF, Beam OFF
    - 3kV follows sw_3kv_enable
    - break early
  end note

  TIMER --> INTERLOCK : (millis() - timerEnterMs) >= 100 ms
  TIMER --> TIMER : otherwise

```
