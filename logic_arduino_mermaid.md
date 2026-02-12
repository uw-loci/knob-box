flowchart TD
    START[setup]
    START ==> INIT[Initialize IO registers, pullups, outputs safe, flags default]
    INIT ==> S0[STATE_INTERLOCK]

    S0 ==> STEP[step]
    S1 ==> STEP
    S2 ==> STEP

    STEP ==> SAMPLE[Sample inputs: switches PB4-PB7, comparators PL0-PL7, ACK PJ1, RESET PJ0]
    SAMPLE ==> DEBOUNCE[Debounce switches PB4-PB7 and reset button]
    DEBOUNCE ==> EDGE[Compute resetButtonEdge = resetButtonDb and not prevResetButtonDb]
    EDGE ==> DISPATCH{currentState}

    DISPATCH ===> INTERLOCK_CASE{STATE_INTERLOCK}
    INTERLOCK_CASE ==> I_3KVI{comparators and 3kV_I}
    I_3KVI ===> I_TO_TIMER[Set state = STATE_3KV_TIMER]
    I_TO_TIMER ==> I_TIMERSET[Set timerEnterMs = millis]
    I_TIMERSET ==> I_OUT_TIMER[Outputs: CCS OFF, Beam OFF, 3kV OFF]
    I_OUT_TIMER ==> FLAGS_AND_OUT

    I_3KVI ===> I_OUT_DEFAULT[Outputs: CCS OFF, Beam OFF, 3kV = sw_3kv_enable]
    I_OUT_DEFAULT ==> I_ENTER_NOMOP{resetButtonEdge and comparators == 0 and sw_arm_80kv and sw_3kv_enable}
    I_ENTER_NOMOP ===> I_TO_NOMOP[Set state = STATE_NOM_OP]
    I_TO_NOMOP ==> FLAGS_AND_OUT
    I_ENTER_NOMOP ===> I_STAY[Stay STATE_INTERLOCK]
    I_STAY ==> FLAGS_AND_OUT

    DISPATCH ===> NOMOP_CASE{STATE_NOM_OP}
    NOMOP_CASE ==> N_3KV{comparators and 3kV_I or 3kV_V}
    N_3KV ===> N_TO_TIMER[Set state = STATE_3KV_TIMER]
    N_TO_TIMER ==> N_TIMERSET[Set timerEnterMs = millis]
    N_TIMERSET ==> N_OUT_TIMER[Outputs: CCS OFF, Beam OFF, 3kV OFF]
    N_OUT_TIMER ==> FLAGS_AND_OUT

    N_3KV ===> N_OTHERFAULT{comparators != 0 or not sw_arm_80kv or not sw_3kv_enable}
    N_OTHERFAULT ===> N_TO_INTERLOCK[Set state = STATE_INTERLOCK]
    N_TO_INTERLOCK ==> N_OUT_INTERLOCK[Outputs: CCS OFF, Beam OFF, 3kV = sw_3kv_enable]
    N_OUT_INTERLOCK ==> FLAGS_AND_OUT

    N_OTHERFAULT ===> N_NORMAL[Outputs: 3kV = sw_3kv_enable, CCS = sw_ccs_allow, Beam = sw_arm_beams]
    N_NORMAL ==> FLAGS_AND_OUT

    DISPATCH ===> TIMER_CASE{STATE_3KV_TIMER}
    TIMER_CASE ==> T_OUT[Outputs: CCS OFF, Beam OFF, 3kV OFF]
    T_OUT ==> T_DONE{millis - timerEnterMs >= 100 ms}
    T_DONE ===> T_TO_INTERLOCK[Set state = STATE_INTERLOCK]
    T_TO_INTERLOCK ==> FLAGS_AND_OUT
    T_DONE ===> T_STAY[Stay STATE_3KV_TIMER]
    T_STAY ==> FLAGS_AND_OUT

    FLAGS_AND_OUT[Set nomOp = currentState == STATE_NOM_OP]
    FLAGS_AND_OUT ==> WRITE_FLAGS[write_flags]
    WRITE_FLAGS ==> ACK_TOGGLE{ACK level changed since last}
    ACK_TOGGLE ===> CLEAR_LATCH[Clear latched flags: comparators=0, switches=0]
    CLEAR_LATCH ==> LATCH_NEW
    ACK_TOGGLE ===> LATCH_NEW[Latch flags: comparators or= sample.comparators, switches or= sample.switchesAssertPortB]
    LATCH_NEW ==> PORTA_BUILD[Build PORTA: PA0-PA2 outputs, PA3 nomOp, PA4-PA7 latched switches]
    PORTA_BUILD ==> PORT_WRITE[Write PORTA and PORTC latched comparator faults]
    PORT_WRITE ==> WRITE_OUTPUTS[write_outputs]
    WRITE_OUTPUTS ==> LOOPBACK[loop calls step again]
    LOOPBACK ==> STEP
