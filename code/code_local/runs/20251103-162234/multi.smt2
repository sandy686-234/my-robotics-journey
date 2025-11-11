(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.013617625, 32.013617625] lhs=13.879793 rhs=32.000000
(assert (! (<= 13.879793266 32.000000000) :named w0))
; window 1: [0.013617625, 33.013617625] lhs=27.773328 rhs=33.000000
(assert (! (<= 27.773327949 33.000000000) :named w1))
; window 2: [0.013617625, 54.013617625] lhs=42.951495 rhs=54.000000
(assert (! (<= 42.951495091 54.000000000) :named w2))
; window 3: [0.013617625, 66.013617625] lhs=58.373218 rhs=66.000000
(assert (! (<= 58.373218032 66.000000000) :named w3))
; window 4: [0.013617625, 76.013617625] lhs=75.003011 rhs=76.000000
(assert (! (<= 75.003011298 76.000000000) :named w4))
; window 5: [0.013617625, 78.013617625] lhs=88.024303 rhs=78.000000
(assert (! (<= 88.024303467 78.000000000) :named w5))
(declare-fun e_A_leg1 () Real)
(declare-fun e_A_leg2 () Real)
(declare-fun e_B_leg1 () Real)
(declare-fun e_B_leg2 () Real)
(assert (! (or (<= (+ e_A_leg1 15.178167 0.100000) e_B_leg1) (<= (+ e_B_leg1 15.421723 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg1))
(assert (! (or (<= (+ e_A_leg1 15.178167 0.100000) e_B_leg2) (<= (+ e_B_leg2 13.021292 0.100000) e_A_leg1)) :named mutex_A_leg1_B_leg2))
(assert (! (or (<= (+ e_A_leg2 16.629793 0.100000) e_B_leg1) (<= (+ e_B_leg1 15.421723 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg1))
(assert (! (or (<= (+ e_A_leg2 16.629793 0.100000) e_B_leg2) (<= (+ e_B_leg2 13.021292 0.100000) e_A_leg2)) :named mutex_A_leg2_B_leg2))
(check-sat)
(get-unsat-core)