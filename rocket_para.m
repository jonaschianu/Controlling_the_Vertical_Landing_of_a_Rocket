%Parameters
m   =  50000;           % Rocket Dry Mass + Fuel mass 25,600Kg + 395,700 Kg
g   =  9.8;             % Gravity m/s^2
Fe  =  m*g;            % Vertical Thrust Force Kn
Fs  =  0;               % Net Lateral Thrust Force (Left thrust - Right thrust)
th  =  0;               % Vertical orientation of rocket
ph  =  0;               % Vertical orientation of vertical thruster
L   = 45;               % L/2 = 𝐿𝑜𝑛𝑑𝑖𝑔𝑢𝑡𝑑𝑖𝑛𝑎𝑙 𝑙𝑒𝑛𝑔𝑡ℎ 𝑏𝑒𝑡𝑤𝑒𝑒𝑛 𝑡ℎ𝑒 𝐶𝑒𝑛𝑡𝑒𝑟 𝑜𝑓 𝐺𝑟𝑎𝑣𝑖𝑡𝑦 (𝐶𝑂𝐺) 𝑎𝑛𝑑 𝐹𝐸 + Nozzle Length m
J   = m*(L)^2/12;       % Inertia of rocket