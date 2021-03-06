%Parameters
m   =  50000;           % Rocket Dry Mass + Fuel mass 25,600Kg + 395,700 Kg
g   =  9.8;             % Gravity m/s^2
Fe  =  m*g;            % Vertical Thrust Force Kn
Fs  =  0;               % Net Lateral Thrust Force (Left thrust - Right thrust)
th  =  0;               % Vertical orientation of rocket
ph  =  0;               % Vertical orientation of vertical thruster
L   = 45;               % L/2 = πΏππππππ’π‘πππππ πππππ‘β πππ‘π€πππ π‘βπ πΆπππ‘ππ ππ πΊπππ£ππ‘π¦ (πΆππΊ) πππ πΉπΈ + Nozzle Length m
J   = m*(L)^2/12;       % Inertia of rocket