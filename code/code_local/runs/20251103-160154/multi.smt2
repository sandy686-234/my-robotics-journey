(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.013987291, 17.013987291] lhs=14.839724 rhs=17.000000
(assert (! (<= 14.839724354 17.000000000) :named w0))
; window 1: [0.013987291, 18.013987291] lhs=29.693190 rhs=18.000000
(assert (! (<= 29.693190126 18.000000000) :named w1))
; window 2: [0.013987291, 34.013987291] lhs=45.831288 rhs=34.000000
(assert (! (<= 45.831288356 34.000000000) :named w2))
; window 3: [0.013987291, 36.013987291] lhs=62.212942 rhs=36.000000
(assert (! (<= 62.212942387 36.000000000) :named w3))
; window 4: [0.013987291, 54.013987291] lhs=79.802667 rhs=54.000000
(assert (! (<= 79.802666741 54.000000000) :named w4))
; window 5: [0.013987291, 56.013987291] lhs=93.783890 rhs=56.000000
(assert (! (<= 93.783889999 56.000000000) :named w5))
(declare-fun e_A_leg1 () Real)
(declare-fun e_A_leg2 () Real)
(declare-fun e_B_leg1 () Real)
(declare-fun e_B_leg2 () Real)
(assert (! (or (<= (+ e_A_leg1 16.138098 0.100000) e_B_leg1) (<= (+ e_B_leg1 16.381654 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg1))
(assert (! (or (<= (+ e_A_leg1 16.138098 0.100000) e_B_leg2) (<= (+ e_B_leg2 13.981223 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg2))
(assert (! (or (<= (+ e_A_leg2 17.589724 0.100000) e_B_leg1) (<= (+ e_B_leg1 16.381654 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg1))
(assert (! (or (<= (+ e_A_leg2 17.589724 0.100000) e_B_leg2) (<= (+ e_B_leg2 13.981223 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg2))
(check-sat)
(get-unsat-core)