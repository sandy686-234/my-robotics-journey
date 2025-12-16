(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.012846375, 17.012846375] lhs=16.188790 rhs=17.000000
(assert (! (<= 16.188790205 17.000000000) :named w0))
; window 1: [0.012846375, 18.012846375] lhs=32.392571 rhs=18.000000
(assert (! (<= 32.392571046 18.000000000) :named w1))
; window 2: [0.012846375, 34.012846375] lhs=49.997769 rhs=34.000000
(assert (! (<= 49.997769116 34.000000000) :named w2))
; window 3: [0.012846375, 36.012846375] lhs=67.868664 rhs=36.000000
(assert (! (<= 67.868664422 36.000000000) :named w3))
; window 4: [0.012846375, 54.012846375] lhs=87.057455 rhs=54.000000
(assert (! (<= 87.057454627 54.000000000) :named w4))
; window 5: [0.012846375, 56.012846375] lhs=102.309698 rhs=56.000000
(assert (! (<= 102.309698180 56.000000000) :named w5))
(declare-fun e_A_leg1 () Real)
(declare-fun e_A_leg2 () Real)
(declare-fun e_B_leg1 () Real)
(declare-fun e_B_leg2 () Real)
(assert (! (or (<= (+ e_A_leg1 17.605198 0.100000) e_B_leg1) (<= (+ e_B_leg1 17.870895 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg1))
(assert (! (or (<= (+ e_A_leg1 17.605198 0.100000) e_B_leg2) (<= (+ e_B_leg2 15.252244 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg2))
(assert (! (or (<= (+ e_A_leg2 19.188790 0.100000) e_B_leg1) (<= (+ e_B_leg1 17.870895 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg1))
(assert (! (or (<= (+ e_A_leg2 19.188790 0.100000) e_B_leg2) (<= (+ e_B_leg2 15.252244 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg2))
(check-sat)
(get-unsat-core)