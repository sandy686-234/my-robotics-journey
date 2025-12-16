(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.018317166, 17.018317166]  lhs=14.839724  rhs=17.000000
(assert (! (<= 14.839724354 17.000000000) :named w0))
; window 1: [0.018317166, 34.018317166]  lhs=30.977823  rhs=34.000000
(assert (! (<= 30.977822585 34.000000000) :named w1))
; window 2: [0.018317166, 54.018317166]  lhs=48.567547  rhs=54.000000
(assert (! (<= 48.567546939 54.000000000) :named w2))
(check-sat)
(get-unsat-core)