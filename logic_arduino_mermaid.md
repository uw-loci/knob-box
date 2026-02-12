flowchart TD
    START[Power up or reset]
    START ==> INIT["Hardware Initialization\nio_init_registers()"]
    INIT ==> DEFAULT["Set default outputs, sample initial inputs"]
    DEFAULT ==> LOOP["step()"]

    LOOP ==> SAMPLE[Sample logic arduino inputs]

    SAMPLE ==> DEBOUNCE[Debounce switches and reset button]

    DEBOUNCE ==> SWITCHES[Compute debounced reset button and switch states]

    SWITCHES ==> STATE{{Current State?}}

    STATE ===> INTERLOCK[STATE_INTERLOCK]
    INTERLOCK ==> I_3KVI{{3kV overcurrent fault?}}
    I_3KVI ==> I_3KVI_T[True]
    I_3KVI ==> I_3KVI_F[False]
    I_3KVI_T ===> I_TO_TIMER[Enter STATE_3KV_TIMER]
    I_3KVI_F ===> I_NOMOP_COND{{Reset button pushed AND comparators safe AND 80kV asserted AND 3kV En asserted}}

    I_NOMOP_COND ==> I_NOMOP_T[True]
    I_NOMOP_COND ==> I_NOMOP_F[False]
    I_NOMOP_T ===> I_TO_NOMOP[Enter STATE_NOM_OP]
    I_NOMOP_F ===> I_STAY_I[Remain STATE_INTERLOCK]

    I_TO_TIMER ==> OUT_ASSIGN
    I_TO_NOMOP ==> OUT_ASSIGN
    I_STAY_I ==> OUT_ASSIGN

    STATE ===> NOMOP[STATE_NOM_OP]
    NOMOP ==> N_3KVFAULT{{3kV I fault OR 3kV V fault?}}
    N_3KVFAULT ==> N_3KV_T[True]
    N_3KVFAULT ==> N_3KV_F[False]
    N_3KV_T ===> N_TO_TIMER[Enter STATE_3KV_TIMER]
    N_3KV_F ===> N_OTHERFAULT{{Any comparator fault OR 80kV switch off OR 3kV En switch off}}

    N_OTHERFAULT ==> N_OTHER_T[True]
    N_OTHERFAULT ==> N_OTHER_F[False]
    N_OTHER_T ===> N_TO_INTERLOCK[Exit to STATE_INTERLOCK]
    N_OTHER_F ===> N_STAY_N[Remain STATE_NOM_OP]

    N_TO_TIMER ==> OUT_ASSIGN
    N_TO_INTERLOCK ==> OUT_ASSIGN
    N_STAY_N ==> OUT_ASSIGN

    STATE ===> TIMER[STATE_3KV_TIMER]
    TIMER ==> T_EXPIRE{{"millis() - timerEnterMs >= 100 ms"}}
    T_EXPIRE ==> T_EXP_T[True]
    T_EXPIRE ==> T_EXP_F[False]
    T_EXP_T ===> T_TO_INTERLOCK[Enter STATE_INTERLOCK]
    T_EXP_F ===> T_STAY_T[Remain STATE_3KV_TIMER]

    T_TO_INTERLOCK ==> OUT_ASSIGN
    T_STAY_T ==> OUT_ASSIGN

    OUT_ASSIGN[Assign outputs based on currentState\nINTERLOCK: CCS = OFF, Beam = OFF, 3kV = switch\n  NOM_OP: CCS = switch, Beam = switch,  3kV = switch\nTIMER: CCS = OFF, Beam = OFF, 3kV = OFF]

    OUT_ASSIGN ==> FLAGS[Update Flags]
    
    FLAGS ==> LOOP
