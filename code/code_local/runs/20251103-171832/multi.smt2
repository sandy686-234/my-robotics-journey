(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.011656666, 32.011656666] lhs=13.355984 rhs=32.000000
(assert (! (<= 13.355983742 32.000000000) :named w0))
; window 1: [0.011656666, 33.011656666] lhs=26.725055 rhs=33.000000
(assert (! (<= 26.725054548 33.000000000) :named w1))
; window 2: [0.011656666, 54.011656666] lhs=41.317585 rhs=54.000000
(assert (! (<= 41.317584839 54.000000000) :named w2))
; window 3: [0.011656666, 66.011656666] lhs=56.142073 rhs=66.000000
(assert (! (<= 56.142073034 66.000000000) :named w3))
; window 4: [0.011656666, 76.011656666] lhs=72.117104 rhs=76.000000
(assert (! (<= 72.117104395 76.000000000) :named w4))
; window 5: [0.011656666, 88.011656666] lhs=84.655468 rhs=88.000000
(assert (! (<= 84.655468045 88.000000000) :named w5))
(declare-fun e_A_leg1 () Real)
(declare-fun e_A_leg2 () Real)
(declare-fun e_B_leg1 () Real)
(declare-fun e_B_leg2 () Real)
(assert (! (or (<= (+ e_A_leg1 14.592530 0.100000) e_B_leg1) (<= (+ e_B_leg1 14.824488 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg1))
(assert (! (or (<= (+ e_A_leg1 14.592530 0.100000) e_B_leg2) (<= (+ e_B_leg2 12.538364 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg2))
(assert (! (or (<= (+ e_A_leg2 15.975031 0.100000) e_B_leg1) (<= (+ e_B_leg1 14.824488 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg1))
(assert (! (or (<= (+ e_A_leg2 15.975031 0.100000) e_B_leg2) (<= (+ e_B_leg2 12.538364 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg2))
(check-sat)
(get-unsat-core)