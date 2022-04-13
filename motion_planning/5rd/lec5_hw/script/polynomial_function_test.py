#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy
T = sympy.Symbol('end_T')

J = 1+T+T**2+T**3+T**4+T**5
J_expr = sympy.simplify(J)
print(J_expr)
print('-------------')
J_dif = sympy.diff(J,T)
# 简化表达式
J_dif_expr = sympy.simplify(J_dif)
print(J_dif_expr)