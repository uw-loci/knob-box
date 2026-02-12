stateDiagram-v2
  direction TB

  %% -------------------------
  %% STATES
  %% -------------------------
  state "STATE_INTERLOCK\n(BI / INTERLOCK)\n\nOutputs:\n  CCS(A0)=OFF\n  Beam(A1)=OFF\n  3kV(A2)=sw_3kv_enable\n\nNomOp flag: LOW" as INTERLOCK

  state "STATE_NOM_OP\n(NOMINAL OP)\n\nOutputs:\n  3kV(A2)=sw_3kv_enable\n  CCS(A0)=sw_ccs_allow\n  Beam(A1)=sw_arm_beams\n\nNomOp flag: HIGH" as NOMOP

  state "STATE_3KV_TIMER\n(3kV TIMER)\n\nOutputs:\n  CCS(A0)=OFF\n  Beam(A1)=OFF\n  3kV(A2)=OFF (forced)\n\nTimer:\n  hold 100 ms\n  then -> INTERLOCK\n\nNomOp flag: LOW" as TIMER

  %% -------------------------
  %% INITIAL
  %% -------------------------
  [*] --> INTERLOCK : setup()\ncurrentState=INTERLOCK

  %% -------------------------
  %% INTERLOCK TRANSITIONS
  %% -------------------------
  INTERLOCK --> TIMER : if (comparators & 3kV_I) != 0\n(3kV overcurrent)\n\nActions:\n  timerEnterMs = millis()\n  enable3kV = OFF\n  (break early)

  INTERLOCK --> NOMOP : if resetButtonEdge\nAND comparators == 0\nAND sw_arm_80kv == 1\nAND sw_3kv_enable == 1\n\n(Enter NomOp only on reset edge + all-safe + required switches)

  INTERLOCK --> INTERLOCK : otherwise\n\nActions every loop:\n  CCS=OFF\n  Beam=OFF\n  3kV = sw_3kv_enable

  %% -------------------------
  %% NOMOP TRANSITIONS
  %% -------------------------
  NOMOP --> TIMER : if (comparators & (3kV_I|3kV_V)) != 0\n(any 3kV V/I trip)\n\nActions:\n  CCS=OFF, Beam=OFF, 3kV=OFF\n  timerEnterMs = millis()\n  (break early)

  NOMOP --> INTERLOCK : else if comparators != 0\nOR sw_arm_80kv == 0\nOR sw_3kv_enable == 0\n\n(Any non-3k comparator trip OR required switch drops)\n\nActions:\n  CCS=OFF\n  Beam=OFF\n  3kV = sw_3kv_enable\n  (break early)

  NOMOP --> NOMOP : else\n\nActions:\n  3kV = sw_3kv_enable\n  CCS = sw_ccs_allow\n  Beam = sw_arm_beams

  %% -------------------------
  %% TIMER TRANSITIONS
  %% -------------------------
  TIMER --> INTERLOCK : if (millis() - timerEnterMs) >= 100 ms

  TIMER --> TIMER : else (still timing)\n\nActions every loop:\n  CCS=OFF\n  Beam=OFF\n  3kV=OFF
