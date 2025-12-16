(set-logic QF_LRA)
(set-option :produce-unsat-cores true)
; window 0: [0.024870833, 17.024870833]  lhs=16.188790  rhs=17.000000
(assert (! (<= 16.188790205 17.000000000) :named w0))
; window 1: [0.024870833, 34.024870833]  lhs=33.793988  rhs=34.000000
(assert (! (<= 33.793988275 34.000000000) :named w1))
; window 2: [0.024870833, 54.024870833]  lhs=52.982778  rhs=54.000000
(assert (! (<= 52.982778479 54.000000000) :named w2))
(check-sat)
(get-unsat-core)