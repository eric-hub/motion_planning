#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy
#非交换 
AA = sympy.Symbol('AA',commutative=False)
AA_T = sympy.Symbol('AA_T',commutative=False)
BB = sympy.Symbol('BB',commutative=False)
BB_T = sympy.Symbol('BB_T',commutative=False)
U = sympy.Symbol('U',commutative=False)
U_T = sympy.Symbol('U_T',commutative=False)
gg = sympy.Symbol('gg',commutative=False)
gg_T = sympy.Symbol('gg_T',commutative=False)

X0 = sympy.Symbol('X0',commutative=False)
X0_T = sympy.Symbol('X0_T',commutative=False)

Xref = sympy.Symbol('Xref',commutative=False)
Xref_T = sympy.Symbol('Xref_T',commutative=False)

Q = sympy.Symbol('Q',commutative=False)
J = (U_T*BB_T+X0_T*AA_T+gg_T-Xref_T)*Q*(BB*U+AA*X0+gg-Xref)


print('-------------')
# 展开多项式
print(sympy.expand(J))