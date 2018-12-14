% motor calculation

Nm = 1.7;

wayNm = 2*pi*1000

wayUp = 4

ratio = wayNm/wayUp

efficiency = 0.4;
efficiency = 0.2;

N_Out = Nm * ratio * efficiency

Kg_Out = N_Out/9.81

