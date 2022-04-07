#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import sympy
T = sympy.Symbol('T')


p_x_0=sympy.Symbol('p_x_0')
p_y_0=sympy.Symbol('p_y_0')
p_z_0=sympy.Symbol('p_z_0')
v_x_0=sympy.Symbol('v_x_0')
v_y_0=sympy.Symbol('v_y_0')
v_z_0=sympy.Symbol('v_z_0')

p_x_f=sympy.Symbol('p_x_f')
p_y_f=sympy.Symbol('p_y_f')
p_z_f=sympy.Symbol('p_z_f')

# 代码中暂指定结束与初始速度相同,先写出来，为了代码完整性
# v_x_f=sympy.Symbol('v_x_f')
# v_y_f=sympy.Symbol('v_y_f')
# v_z_f=sympy.Symbol('v_z_f')
# 末状态的速度为0，参数太多了
v_x_f=0.0
v_y_f=0.0
v_z_f=0.0

delta_p_x= p_x_f - v_x_0 * T - p_x_0
delta_p_y= p_y_f - v_y_0 * T - p_y_0
delta_p_z= p_z_f - v_z_0 * T - p_z_0
delta_v_x= v_x_f - v_x_0
delta_v_y= v_y_f - v_y_0
delta_v_z= v_z_f - v_z_0

alpha1 = -(12/T**3)*delta_p_x + (6/T**2)*delta_v_x
alpha2 = -(12/T**3)*delta_p_y + (6/T**2)*delta_v_y
alpha3 = -(12/T**3)*delta_p_z + (6/T**2)*delta_v_z
beta1 = (6/T**2)*delta_p_x - (2/T)*delta_v_x
beta2 = (6/T**2)*delta_p_y - (2/T)*delta_v_y
beta3 = (6/T**2)*delta_p_z - (2/T)*delta_v_z
J = T+((1/3)*alpha1**2*T**3 + alpha1*beta1*T**2 + beta1**2*T)+((1/3)*alpha2**2*T**3 + alpha2*beta2*T**2 + beta2**2*T)+((1/3)*alpha3**2*T**3 + alpha3*beta3*T**2 + beta3**2*T)
J_expr = sympy.simplify(J)
print(J_expr)
# 1.0*(1.0*T**4 + 4.0*T**2*v_x_0**2 + 4.0*T**2*v_y_0**2 + 4.0*T**2*v_z_0**2 + 12.0*T*p_x_0*v_x_0 - 12.0*T*p_x_f*v_x_0 + 12.0*T*p_y_0*v_y_0 - 12.0*T*p_y_f*v_y_0 + 12.0*T*p_z_0*v_z_0 - 12.0*T*p_z_f*v_z_0 + 12.0*p_x_0**2 - 24.0*p_x_0*p_x_f + 12.0*p_x_f**2 + 12.0*p_y_0**2 - 24.0*p_y_0*p_y_f + 12.0*p_y_f**2 + 12.0*p_z_0**2 - 24.0*p_z_0*p_z_f + 12.0*p_z_f**2)/T**3
print('-------------')
J_dif = sympy.diff(J,T)
# 简化表达式
J_dif_expr = sympy.simplify(J_dif)
print(J_dif_expr)
# print('-------------')
# 1.0*(1.0*T**4 - 4.0*T**2*v_x_0**2 - 4.0*T**2*v_y_0**2 - 4.0*T**2*v_z_0**2 - 24.0*T*p_x_0*v_x_0 + 24.0*T*p_x_f*v_x_0 - 24.0*T*p_y_0*v_y_0 + 24.0*T*p_y_f*v_y_0 - 24.0*T*p_z_0*v_z_0 + 24.0*T*p_z_f*v_z_0 - 36.0*p_x_0**2 + 72.0*p_x_0*p_x_f - 36.0*p_x_f**2 - 36.0*p_y_0**2 + 72.0*p_y_0*p_y_f - 36.0*p_y_f**2 - 36.0*p_z_0**2 + 72.0*p_z_0*p_z_f - 36.0*p_z_f**2)/T**4
# 漂亮地打印输出
# sympy.pprint(expr)
# print(J_dif.evalf())