#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy
T = sympy.Symbol('T')


x_1=sympy.Symbol('x_1')
A_0=sympy.Symbol('A_0')
x_0=sympy.Symbol('x_0')
B_0=sympy.Symbol('B_0')
u_0=sympy.Symbol('u_0')
g_0=sympy.Symbol('g_0')

x_1 = A_0*x_0+B_0*u_0+g_0

A_1=sympy.Symbol('A_1')
B_1=sympy.Symbol('B_1')
u_1=sympy.Symbol('u_1')
g_1=sympy.Symbol('g_1')


x_2 = A_1*x_1+B_1*u_1+g_1

print('-------------')
print(sympy.simplify(x_2))