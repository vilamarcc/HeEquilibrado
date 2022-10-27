% ------------------------------------------------------------------
% PRÁCTICA DE EQUILIBRADO : APARTADO 1.2.3
% ------------------------------------------------------------------
% Vuelo de ascenso

clear,clc
w_dim_orig = linspace(-25,10,36);
w_dim = w_dim_orig;
r2d = 180/pi;
atm = getISA;
hsl = 0;
he  = PadfieldBo105(atm);

rho0 = atm.density(hsl);
Rad = he.mainRotor.R;
Omega_N = he.mainRotor.Omega;
ndV = w_dim / (Rad*Omega_N);

muWT          = [0; 0; 0];


options       = setHeroesRigidOptions;
for i=1:length(ndV)
    
     FC            = {'VOR', ndV(i),...
                     'Psi',0,...
                     'uTOR',0,...
                     'cs',0,...
                     'vTOR',0};

    ndHe           = rigidHe2ndHe(he,atm,hsl);            
    ndts           = getNdHeTrimState(ndHe,muWT,FC,options);
    ts             = ndHeTrimState2HeTrimState(ndts,he,atm,hsl);
    lambda_i (i)   = ndts.solution.lambda0;
    v_i (i)        = lambda_i(i) * Omega_N*Rad;
    Pm  (i)        = ts.Pow.PM;
    T_0 (i)        = ts.solution.T0;
    Theta_0 (i)   = ts.solution.theta0;
    Theta_0tr (i) = ts.solution.theta0tr;
    vi_vi0 (i) = v_i(i)*(T_0(i)/(2*rho0*pi*Rad^2))^(-0.5);
    Vv_vi0 (i) = w_dim_orig(i)*(T_0(i)/(2*rho0*pi*Rad^2))^(-0.5);
    
end

figure(1)
grid minor
hold on
plot(w_dim, lambda_i)
xlabel('$$w$$ [m/s]', 'Interpreter', 'latex'); ylabel('$$\lambda_{i}$$ [-]', 'Interpreter', 'latex')

figure(2)
grid minor
hold on
plot(w_dim, Theta_0*r2d, w_dim, Theta_0tr*r2d)
xlabel('$$w$$ [m/s]', 'Interpreter', 'latex'); ylabel('$$\theta$$ [deg]', 'Interpreter', 'latex')
legend('$$\theta_{0}$$', '$$\theta_{TR}$$', 'Interpreter', 'latex', 'Location', 'best');

figure(3)
grid minor
hold on
plot(w_dim, Pm)
xlabel('$$w$$ [m/s]', 'Interpreter', 'latex'); ylabel('$$P_M [W]$$', 'Interpreter', 'latex')

figure(4)
grid minor
hold on
plot(Vv_vi0, -vi_vi0)
xlabel('$$\frac{V_{V}}{v_{i0}}$$ [-]', 'Interpreter', 'latex'); ylabel('$$\frac{v_{i}}{v_{i0}}$$ [-]', 'Interpreter', 'latex')
ylim([0 2]);
