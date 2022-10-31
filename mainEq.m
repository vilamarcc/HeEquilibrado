%% MAIN ----------------------------------------------------------------------

%% SETUP PREELIMINAR DEL HE
close all; 
clear,clc

% Creamos el he Bo105 a punto fijo y nivel del mar

hsl               = 0;
atm               = getISA;
heRef             = PadfieldBo105(atm);
ndHeRef           = rigidHe2ndHe(heRef,atm,hsl);
GA                = [0; 0; -1];
muW               = [0; 0; 0];
opt               = optimset('Display','off');
options           = setHeroesRigidOptions;
options.armonicInflowModel = @none;
ndRotorRef        = ndHeRef.mainRotor;

% default optiosns are set
options = setHeroesRigidOptions;

%engineState
%options.engineState = @mainShaftBroken;

% uniformInflowModel options are updated with
% @Cuerva model for induced velocity
options.uniformInflowModel = @Cuerva;
options.armonicInflowModel = @none;

% engineState options are updated for the 
%options.engineState = @EngineOffTransmissionOn;

% mrForces options are updated for 
%options.mrForces = @completeF;
options.mrForces =@thrustF;

% trForces options are updated
options.trForces = @completeF;

%% 1) Análisis de la condición de referencia en vuelo a punto fijo a nivel del mar.

close all; 
clear,clc
disp("1) Análisis de la condición de referencia en vuelo a punto fijo a nivel del mar.");

% 1) Determinar la solución al problema de equilibrado en la condición de vuelo a punto
% fijo para el helicóptero Bo105. Esta solución consiste en determinar los valores de los
% parámetros: ángulo de balanceo Φ, ángulo de cabeceo, Θ, velocidad inducida del rotor
% principal, λi , velocidad inducida del rotor antipar, λia, colectivo del rotor principal, θ0,
% paso cíclico lateral θ1C, paso cíclico longitudinal, θ1S, colectivo del rotor antipar, θT , ángulo
% de conicidad, β0, ángulo de batimiento longitudinal, β1C, ángulo de batimiento lateral,
% β1S, y potencia necesaria para el vuelo, PM.

% 2) En esta situación de vuelo debe determinarse la relación entre la potencia neta consumida
% por el rotor antipar, Pra, y la consumida por el rotor principal, Prp

% 3 )Analice la contribución de cada elemento activo (rotor principal, rotor antipar, fuselaje,
% estabilizador horizontal derecho, estabilizador horizontal izquierdo y estabilizador vertical) a las fuerzas globales y momentos globales en torno al centro de masas del vehículo.
% Se plantea hacer un análisis similar al presentado en las tablas 6.1 y 6.2 de Cuerva Tejero
% et al. [2008].

hsl = 0;

[heRef, ndHeRef, atm,options] = Utils.createBo105();

muWT = [0; 0; 0];

FC = {'VOR',10^-4,...
      'betaf0',0,...
      'gammaT',0,...
      'cs',0,...
      'vTOR',0};

tic
ndTrimState = getNdHeTrimState(ndHeRef,muWT,FC,options);
dimTrimState = ndHeTrimState2HeTrimState(ndTrimState,heRef,atm,hsl);
toc


% Muestra en pantalla el resultado de los angulos de trimado asi como las
% contribuciones de elementos activos de manera porcentual
Utils.displayTrimState(ndTrimState,dimTrimState.Pow.PM);


%% 2) Análisis dimensional del vuelo a punto fijo de un helicóptero

close all; 
clear,clc
disp("2) Análisis dimensional del vuelo a punto fijo de un helicóptero");

% 1. Aplicar el teorema PI de Buckgingham del análisis dimensional eligiendo como variables
% dimensionalmente independientes el radio del rotor R, la velocidad angular del rotor,
% Ω, y la densidad del aire a la altitud de vuelo, ρ, obteniendo el coeficiente de potencia
% adimensional en función de los correspondientes parámetros adimensionales.

% 2. Interpretar físicamente los parámetros adimensionales obtenidos.

% 3. Obtener la expresión de la potencia normalizada, referida o pseudo-adimensional considerando las variables que permanecen constantes de un ensayo de vuelo a otro.

% 4. Particularizar la expresión de la potencia normalizada para el caso de vuelo a punto fijo,
% V = 0 y obtener expresiones funcionales normalizadas para los siguientes casos:
% a) Altura de vuelo Z/R >> 2 y efectos de compresibilidad despreciables.
% b) Altura de vuelo Z/R >> 2 y efectos de compresibilidad importantes.
% c) Altura de vuelo Z/R ≤ 2 y efectos de compresibilidad despreciables.


% 5. Empleando el modelo de la energía obtener la expresión de la potencia necesaria normalizada para vuelo a punto fijo. Para ello se procede de la siguiente forma:
% a) Obtener una expresión de la potencia necesaria para vuelo a punto fijo como la
% suma de la potencia inducida para sustentar el peso de la aeronave corregida con el
% factor κ, y la potencia parásita del rotor considerando un coeficiente de resistencia
% promediado, cd0 constante.
% b) Adimensionalizando la anterior expresión obtener el coeficiente de potencia adimensional para vuelo a punto fijo, CQ.
% c) Transformar la anterior expresión a variables normalizadas haciendo explícita la dependencia funcional de la potencia normalizada con las variables normalizadas.
% d) Identificar el tipo de dependencia funcional obtenida mediante su representación gráfica.
% e) Discutir la equivalencia entre la dependencia funcional obtenida en 5.c) y las obtenidas en 4.a), 4.b) y 4.c)
% f) Analizar como se modifica la expresión obtenida en 5.c) ante variaciones del parámetro κ y cd0.

% 6. Empleando la toolbox de matlab heroes se pretende realizar una simulación de los ensayos en vuelo necesarios para caracterizar la potencia necesaria para el vuelo a punto fijo del
% helicóptero asignado a cada grupo de prácticas. Para ello, se consideran una matriz de
% ensayos en vuelo a punto fijo definida para 11 pesos diferentes y 5 altitudes de vuelo
% distintas.
% a) Determinar para cada combinación de peso y altitud de la matriz de ensayos la
% potencia necesaria para el vuelo a punto fijo.
% b) Representar en una gráfica la potencia necesaria en función del peso para las diferentes altitudes de vuelo. Comentar los resultados obtenidos.
% c) Representar en una gráfica la potencia referida, P/(σIω3) en función del peso referido W/(σIω2). Comentar los resultados obtenidos.
% d) Representar en una gráfica la potencia referida, P/(σIω)
% 3) en función del peso referido elevado a 3/2, [W/(σIω2)]3/2. Comentar los resultados obtenidos.
% e) Discutir los resultados obtenidos en los apartados 6.b), 6.c) y 6.d).
% f) Teniendo en cuenta los resultados teóricos del apartado 5 identificar y proponer valores para los parámetros κ y cd0 del helicóptero analizado.

[heRef, ~, atm,options] = Utils.createBo105();
muWT = [0; 0; 0];

Ws = linspace(0.8,1.2,10)*heRef.inertia.W;
hs = linspace(0,5000,5);

heRefs = getParametricCellHe(heRef,'inertia.W',Ws); 

PMs = zeros(length(hs),length(Ws));
syms = ["-","--",":","-.","--o"];

FC = {'VOR',10^-4,...
      'betaf0',0,...
      'gammaT',0,...
      'cs',0,...
      'vTOR',0};


for i = 1:length(hs)
    for j = 1:length(Ws)

        ndHeRef = rigidHe2ndHe(heRefs{j},atm,hs(i));
        ndTrimState_j = getNdHeTrimState(ndHeRef,muWT,FC,options);
        dimTrimState_j = ndHeTrimState2HeTrimState(ndTrimState_j,heRefs{j},atm,hs(i));
        PMs(i,j) = dimTrimState_j.Pow.PM;
        disp("PM = " + string(dimTrimState_j.Pow.PM) + " at h = " + string(hs(i)) + " m, with W = " + string(Ws(j)) + " N");
        
    end
end

figure(1); 
plot(Ws,PMs(1,:)*0.001,syms(1),'Color','black','LineWidth',0.75); hold on
plot(Ws,PMs(2,:)*0.001,syms(2),'Color','black','LineWidth',0.75); hold on
plot(Ws,PMs(3,:)*0.001,syms(3),'Color','black','LineWidth',0.75); hold on
plot(Ws,PMs(4,:)*0.001,syms(4),'Color','black','LineWidth',0.75); hold on
plot(Ws,PMs(5,:)*0.001,syms(5),'Color','black','LineWidth',0.75); hold on
X = [0.55 0.45]; Y = [0.30 0.58];
annotation('textarrow',X,Y,'FontSize',3,'Linewidth',0.5);
annotation('textbox',[.41 .36 .3 .33],'EdgeColor','none','String','$h \uparrow$','FontSize',24,'Linewidth',5,'Interpreter','latex');
xlabel("$W$ [N]",'FontSize',20);
ylabel("$P_{M}$ [kW]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/PMvsW_PF",'eps');

figure(2); 
plot(Ws*(atm.density(0)/atm.density(hs(1))),PMs(1,:)*(atm.density(0)/atm.density(hs(1))),syms(1),'Color','black','LineWidth',0.75); hold on
plot(Ws*(atm.density(0)/atm.density(hs(2))),PMs(2,:)*(atm.density(0)/atm.density(hs(2))),syms(2),'Color','black','LineWidth',0.75); hold on
plot(Ws*(atm.density(0)/atm.density(hs(3))),PMs(3,:)*(atm.density(0)/atm.density(hs(3))),syms(3),'Color','black','LineWidth',0.75); hold on
plot(Ws*(atm.density(0)/atm.density(hs(4))),PMs(4,:)*(atm.density(0)/atm.density(hs(4))),syms(4),'Color','black','LineWidth',0.75); hold on
plot(Ws*(atm.density(0)/atm.density(hs(5))),PMs(5,:)*(atm.density(0)/atm.density(hs(5))),syms(5),'Color','black','LineWidth',0.75); hold on
xlabel("$W/\sigma_I\omega^3$",'FontSize',20);
ylabel("$P_{M}/\sigma_I\omega^3$",'FontSize',20);
grid minor
saveas(gcf,"Graficas/PMsigmavsWsigma_PF",'eps');

figure(3); 
plot((Ws*(atm.density(0)/atm.density(hs(1)))).^(3/2),PMs(1,:)*(atm.density(0)/atm.density(hs(1))),syms(1),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(2)))).^(3/2),PMs(2,:)*(atm.density(0)/atm.density(hs(2))),syms(2),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(3)))).^(3/2),PMs(3,:)*(atm.density(0)/atm.density(hs(3))),syms(3),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(4)))).^(3/2),PMs(4,:)*(atm.density(0)/atm.density(hs(4))),syms(4),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(5)))).^(3/2),PMs(5,:)*(atm.density(0)/atm.density(hs(5))),syms(5),'Color','black','LineWidth',0.75); hold on
xlabel("$(W/\sigma_I\omega^3)^{\frac{3}{2}}$",'FontSize',20);
ylabel("$P_{M}/\sigma_I\omega^3$",'FontSize',20);
grid minor
saveas(gcf,"Graficas/PMsigmavsWsigma32_PF",'eps');

figure(4); 
plot((Ws*(atm.density(0)/atm.density(hs(1)))).^(3/2),PMs(1,:)*(atm.density(0)/atm.density(hs(1))),syms(1),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(2)))).^(3/2),PMs(2,:)*(atm.density(0)/atm.density(hs(2))),syms(2),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(3)))).^(3/2),PMs(3,:)*(atm.density(0)/atm.density(hs(3))),syms(3),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(4)))).^(3/2),PMs(4,:)*(atm.density(0)/atm.density(hs(4))),syms(4),'Color','black','LineWidth',0.75); hold on
plot((Ws*(atm.density(0)/atm.density(hs(5)))).^(3/2),PMs(5,:)*(atm.density(0)/atm.density(hs(5))),syms(5),'Color','black','LineWidth',0.75); hold on

c1s = zeros(1,length(hs));
c0s = zeros(1,length(hs));

for j = 1:length(hs)

    regress = polyfit((Ws*(atm.density(0)/atm.density(hs(j)))).^(3/2),PMs(j,:)*(atm.density(0)/atm.density(hs(j))),1);
    c1s(j) = regress(1);
    c0s(j) = regress(2);

end

c1 = mean(c1s);
c0 = mean(c0s);

[k,cdo] = Utils.getkcdoDependency(heRef,c1,c0,atm.density(hs(3))/atm.density(0));

reg = @(x) c1*x + c0;

Ws_reg = linspace(0,8*10^6,10);

h = plot(Ws_reg,reg(Ws_reg),'--','Color','red','LineWidth',0.7); hold on
scatter(0,reg(0),'MarkerEdgeColor','black','MarkerFaceColor','black'); hold on

plot([Ws_reg(3),Ws_reg(4)],[reg(Ws_reg(3)), reg(Ws_reg(3))],'--','Color','black','LineWidth',0.5); hold on
plot([Ws_reg(4),Ws_reg(4)],[reg(Ws_reg(3)), reg(Ws_reg(4))],'--','Color','black','LineWidth',0.5); hold on

annotation('textbox',[.15 .1 .3 .15],'EdgeColor','none','String','$C_0$','FontSize',24,'Linewidth',5,'Interpreter','latex');
annotation('textbox',[.35 .075 .3 .3],'EdgeColor','none','String','$C_1$','FontSize',24,'Linewidth',5,'Interpreter','latex');

legend(h,"Ajuste",'Location','southeast','FontSize',22);

xlabel("$(W/\sigma_I\omega^3)^{\frac{3}{2}}$",'FontSize',20);
ylabel("$P_{M}/\sigma_I\omega^3$",'FontSize',20);
grid minor
saveas(gcf,"Graficas/PMsigmavsWsigma32_PF_2",'eps2c');

disp("2.6) f) --------------------------------------------------------");
disp("C1 = " + string(c1) + ", C0 = " + string(c0));
disp("Kappa = " + string(k));
disp("Cd0 = " + string(cdo));
disp("-----------------------------------------------------------------");

%% 3)  Análisis de la condición de vuelo vertical.

close all;
clear,clc
disp("3)  Análisis de la condición de vuelo vertical.");

% Se debe analizar una condición de vuelo vertical en equilibrio para las velocidades VV ∈
% [−25, 10] m/s. Especifique con claridad los parámetros que definen la condición de vuelo que
% son usados como entrada para las simulaciones, identificando las dificultades encontradas.

% 1. Representar las siguientes variables en función de la velocidad de vuelo vertical: parámetro
% de velocidad inducida del rotor principal, λi, colectivo del rotor principal, θ0, colectivo del
% rotor antipar, θT y potencia necesaria, PM. Realice los análisis y extraiga las conslusiones
% oportunas.

% 2. Represente la relación vi/vi0 = vi[T/(2ρS)]^−0.5 función de VV /vi0 = VV [T/(2ρS)]^−0.5
% , establezca las oportunas conclusiones sobre esta representación en relación a los resultados
% conocidos de teoría.

h = 0;

[heRef, ndHeRef, atm,options] = Utils.createBo105();
[heRef_Rand, ndHeRef_Rand, atm_Rand,options_Rand] = Utils.createBo105(h,false,NaN,'Rand');
[heRef_Glaubert, ndHeRef_Glaubert, atm_Glaubert,options_Glaubert] = Utils.createBo105(h,false,NaN,'Glauert');

Vvs = linspace(-25,10,36); 

ndVvs = Vvs/(heRef.mainRotor.R*heRef.mainRotor.Omega);

muWT = [0; 0; 0];

FC            = {'VOR', ndVvs,...
                 'Psi',0,...
                 'uTOR',0,...
                 'cs',0,...
                 'vTOR',0};

tic
ndTrimState_vv_i = getNdHeTrimState(ndHeRef,muWT,FC,options);
dimTrimState_vv_i = ndHeTrimState2HeTrimState(ndTrimState_vv_i,heRef,atm,h);
toc
tic
% ndTrimState_vv_i_Rand =
% getNdHeTrimState(ndHeRef_Rand,muWT,FC,options_Rand); %RAND NO FUNCIONA
% dimTrimState_vv_i_Rand = ndHeTrimState2HeTrimState(ndTrimState_vv_i_Rand,heRef_Rand,atm,h);
toc
tic
ndTrimState_vv_i_Glauert = getNdHeTrimState(ndHeRef_Glaubert,muWT,FC,options_Glaubert);
dimTrimState_vv_i_Glauert = ndHeTrimState2HeTrimState(ndTrimState_vv_i_Glauert,heRef_Glaubert,atm,h);
toc

FC_glauert2    = {'VOR', ndVvs(end),...
                 'Psi',0,...
                 'uTOR',0,...
                 'cs',0,...
                 'vTOR',0};

ndTS_aux = getNdHeTrimState(ndHeRef_Glaubert,muWT,FC_glauert2,options_Glaubert);

[heRef_Glaubert_2, ndHeRef_Glaubert_2, atm_Glaubert_2,options_Glaubert_2] = Utils.createBo105(h,false,ndTS_aux.solution,'Glauert');

tic
%ndTrimState_vv_i_Glauert_2 = getNdHeTrimState(ndHeRef_Glaubert_2,muWT,FC,options_Glaubert_2);
%dimTrimState_vv_i_Glauert_2 = ndHeTrimState2HeTrimState(ndTrimState_vv_i_Glauert_2,heRef_Glaubert_2,atm,h);
toc

lambda_i_vvs = zeros(length(Vvs),1);
theta_0_vvs = zeros(length(Vvs),1);
theta_T_vvs = zeros(length(Vvs),1);
PM_vvs = zeros(length(Vvs),1);
v_i_vio = zeros(length(Vvs),1);
Vv_vio = zeros(length(Vvs),1);

%v_i_vio_Rand = zeros(length(Vvs),1);
v_i_vio_Glauert = zeros(length(Vvs),1);
%v_i_vio_Glauert_2 = zeros(length(Vvs),1);

for i = 1:length(Vvs)
    
    lambda_i_vvs(i) = ndTrimState_vv_i.solution.lambda0(i);
    theta_0_vvs(i) = ndTrimState_vv_i.solution.theta0(i);
    theta_T_vvs(i) = ndTrimState_vv_i.solution.theta0tr(i);
    PM_vvs(i) = dimTrimState_vv_i.Pow.PM(i);

    v_io = (dimTrimState_vv_i.solution.T0(i)/(atm.density(0)*2*pi*(heRef.mainRotor.R^2)))^0.5;
    %v_io_Rand = (dimTrimState_vv_i_Rand.solution.T0(i)/atm.density(0)*2*pi*(heRef.mainRotor.R^2))^0.5;
    v_io_Glauert = (dimTrimState_vv_i_Glauert.solution.T0(i)/atm.density(0)*2*pi*(heRef.mainRotor.R^2))^0.5;
    
    v_i_vio(i) = (ndTrimState_vv_i.solution.lambda0(i)*heRef.mainRotor.R*heRef.mainRotor.Omega)/v_io;
    %v_i_vio_Rand(i) = (ndTrimState_vv_i_Rand.solution.lambda0(i)*heRef.mainRotor.R*heRef.mainRotor.Omega)/v_io;
    v_i_vio_Glauert(i) = (ndTrimState_vv_i_Glauert.solution.lambda0(i)*heRef.mainRotor.R*heRef.mainRotor.Omega)/v_io;
    %v_i_vio_Glauert_2(i) = (ndTrimState_vv_i_Glauert_2.solution.lambda0(i)*heRef.mainRotor.R*heRef.mainRotor.Omega)/v_io;

    Vv_vio(i) = Vvs(i)/v_io;

end


LW = 1.5;

syms = ["-","--",":","-.","--o"];


figure(5)
    yyaxis left
        plot(Vvs,lambda_i_vvs,syms(1),'Color','red', 'MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
        ylabel("$\lambda_i [-]$",'FontSize',20);
    hold all
    yyaxis right
        plot(Vvs,PM_vvs*0.001,syms(2),'Color','blue','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot([Vvs(1) Vvs(end)],[0 0],syms(2),'Color','black','LineWidth',0.7); hold on
        %plot([0 0],[PM_vvs(1)*1.2 PM_vvs(end)*1.3],syms(2),'Color','black','LineWidth',0.5); hold on
        scatter(-19.5,0,'filled','MarkerEdgeColor','blue','MarkerFaceColor','blue');
        annotation('textbox',[0.17 0.08 .4 .4],'EdgeColor','none','String','$A$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
        set(gca, 'XColor','k', 'YColor','k')%,'FontSize', 13)
        ylabel("$P_M$ [kW]",'FontSize',20);
    xlabel("$V_V$ [m/s]",'FontSize',20);
    xlim([Vvs(1) Vvs(end)])
    legend("$\lambda_i$","$P_M$",'Location','north', 'NumColumns',1, 'FontSize',20);
    set(gcf, 'Color',[1 1 1]);
    grid minor
    hold off
    saveas(gcf,"Graficas/lambdaivsVv_VV",'eps2c');

figure(6)
    plot(Vvs,theta_0_vvs*(180/pi),syms(1),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(Vvs,theta_T_vvs*(180/pi),syms(2),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    legend("$\theta_0$","$\theta_T$",'Location','best','FontSize',20);
    ylabel("$\theta_T,\theta_0 [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_V$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetavsVv_VV",'eps');

%%
figure(8)
    h = zeros(2,1);
    h(1) = plot(Vv_vio,-v_i_vio,syms(1),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(Vv_vio,-v_i_vio_Glauert,syms(2),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    %plot(Vv_vio,-v_i_vio_Glauert_2,syms(2),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    %scatter(Vv_vio,-v_i_vio_Rand,'MarkerEdgeColor','black'); hold on
    xline(0,'--','Color','black','LineWidth',0.7); hold on
    xline(-2,'--','Color','black','LineWidth',0.7); hold on

    glau = @(x) -0.5.*(x + sqrt((x.^2) -4));
    glau_2 = @(x) -0.5.*(x - sqrt((x.^2) -4));
    vio_aux = linspace(0.85,2,10);
    plot(Vv_vio(1:5),glau(Vv_vio(1:5)),syms(2),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(Vv_vio(1:5),glau_2(Vv_vio(1:5)),syms(2),'Color','black','MarkerIndices',5:5:length(Vvs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot([-3, 0],[3,0],'-.','Color','black','LineWidth',0.7); hold on
    plot([-3, 0],[1.5,0],'-.','Color','black','LineWidth',0.7); hold on

    annotation('textbox',[.7 .45 .4 .4],'EdgeColor','none','String','Ascenso','FontSize',12,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.25 .02 .4 .2],'EdgeColor','none','String','Molinete Frenante','FontSize',12,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.35 .45 .4 .4],'EdgeColor','none','String','Anillos Turbillonarios','FontSize',12,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.24 .22 .4 .2],'EdgeColor','none','String','Estela Turbulenta','FontSize',12,'Linewidth',5,'Interpreter','latex'); hold on


    xlim([-2.5 1]);
    ylabel("$v_i/v_io [-]$",'FontSize',14);
    xlabel("$V_V/v_io [-]$",'FontSize',14);
    legend(h,"Cuerva","Glauert",'Location','east','FontSize',12);
    grid minor
    saveas(gcf,"Graficas/v_ivsVv_VV",'eps');

%% 4) Análisis de la condición de vuelo equilibrado en autorrotación vertical

close all;
clear,clc
disp("4) Análisis de la condición de vuelo equilibrado en autorrotación vertical");

% Se debe analizar una condición de autorrotación vertical, para ello debe establecer cual es
% la condición correspondiente asociada a la potencia que va a definir su condición de modelización. Especifique con claridad la definición utilizada en la simulación de la condición de vuelo,
% identificando las dificultades encontradas.

% 1. Se debe obtener y representar, en función de la velocidad de descenso, VV , las variables:
% parámetro de velocidad inducida del rotor principal, λi, 
% colectivo del rotor principal, θ0 y
% colectivo del rotor antipar, θT . 
% Realice los análisis y extraiga las conslusiones oportunas.

% Extraemos condición inicial para calcular autorrotacion

h = 0;

[heRef, ndHeRef, atm, options] = Utils.createBo105(h);
muWT = [0; 0; 0];

FC_prem         = {'VOR', 10^-4,...
                 'Psi',0,...
                 'gammaT',0,...
                 'cs',0,...
                 'vTOR',0};

Vv_vio = 1.732;
disp( "-----------------------------------------------------");
disp("Calculando punto fijo.....");
tic
ndTrimState_prem = getNdHeTrimState(ndHeRef,muWT,FC_prem,options);
dimTrimState_prem = ndHeTrimState2HeTrimState(ndTrimState_prem,heRef,atm,h);
toc

v_io = (dimTrimState_prem.solution.T0/(atm.density(0)*2*pi*(heRef.mainRotor.R^2)))^0.5;

ndVv = (Vv_vio*v_io)/(heRef.mainRotor.R*heRef.mainRotor.Omega);

FC_0            = {'VOR', ndVv,...
                 'Psi',0,...
                 'gammaT',-pi/2,...
                 'cs',0,...
                 'vTOR',0};


disp("Calculando condición incial.....");
tic
ndTrimState_prem = getNdHeTrimState(ndHeRef,muWT,FC_0,options);
dimTrimState_prem = ndHeTrimState2HeTrimState(ndTrimState_prem,heRef,atm,h);
toc

disp( "-----------------------------------------------------");

% AUTORROTACIÖN

[heRef, ndHeRef, atm, options_MSB] = Utils.createBo105(0,true,ndTrimState_prem.solution);

o_on = linspace(0.8,1.2,36);

FC_atr         = {'omega', o_on,...
                 'Psi',0,...
                 'gammaT',-pi/2,...
                 'cs',0,...
                 'vTOR',0};

disp("Calculado condición de autorrotación......");
tic
ndTrimState_atr_MSB = getNdHeTrimState(ndHeRef,muWT,FC_atr,options_MSB);
dimTrimState_atr_MSB = ndHeTrimState2HeTrimState(ndTrimState_atr_MSB,heRef,atm,h);
toc

[~, ~, ~, options_EOFF] = Utils.createBo105(0,true,ndTrimState_prem.solution);

options_EOFF.engineState = @EngineOffTransmissionOn;

tic
ndTrimState_atr_EOFF = getNdHeTrimState(ndHeRef,muWT,FC_atr,options_EOFF);
dimTrimState_atr_EOFF = ndHeTrimState2HeTrimState(ndTrimState_atr_MSB,heRef,atm,h);
toc

Vvs_atr_MSB = zeros(length(o_on),1); Vvs_atr_EOFF = zeros(length(o_on),1);
lamdas_i_MSB = zeros(length(o_on),1); lamdas_i_EOFF = zeros(length(o_on),1);
theta0s_i_MSB = zeros(length(o_on),1); theta0s_i_EOFF = zeros(length(o_on),1);
thetaTs_i_MSB = zeros(length(o_on),1); thetaTs_i_EOFF = zeros(length(o_on),1);
Omegas_i_MSB = zeros(length(o_on),1); Omegas_i_EOFF = zeros(length(o_on),1);

CMys_0 = zeros(length(o_on),2); CMys_T = zeros(length(o_on),2);

for i = 1:length(o_on)

    Vvs_atr_MSB(i) = -dimTrimState_atr_MSB.solution.wT(i);
    Vvs_atr_EOFF(i) = -dimTrimState_atr_EOFF.solution.wT(i);

    lamdas_i_MSB(i) = ndTrimState_atr_MSB.solution.lambda0(i);
    theta0s_i_MSB(i) = ndTrimState_atr_MSB.solution.theta0(i);
    thetaTs_i_MSB(i) = ndTrimState_atr_MSB.solution.theta0tr(i);
    Omegas_i_MSB(i) = dimTrimState_atr_MSB.solution.Omega(i);

    lamdas_i_EOFF(i) = ndTrimState_atr_EOFF.solution.lambda0(i);
    theta0s_i_EOFF(i) = ndTrimState_atr_EOFF.solution.theta0(i);
    thetaTs_i_EOFF(i) = ndTrimState_atr_EOFF.solution.theta0tr(i);
    Omegas_i_EOFF(i) = dimTrimState_atr_MSB.solution.Omega(i);

    CMys_0(i,1) = ndTrimState_atr_MSB.actions.mainRotor.fuselage.CMz(i); 
    CMys_0(i,2) = ndTrimState_atr_EOFF.actions.mainRotor.fuselage.CMz(i);

    CMys_T(i,1) = ndTrimState_atr_MSB.actions.tailRotor.fuselage.CMFz(i); 
    CMys_T(i,2) = ndTrimState_atr_EOFF.actions.tailRotor.fuselage.CMFz(i);

end


syms = ["-","--","-o","--o","-^","--^"];
LW = 1.5;
figure(9)
    yyaxis left
        plot(Vvs_atr_MSB,lamdas_i_MSB,syms(2),'Color','Black', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'Black', 'LineWidth',0.7, 'MarkerSize', 4); hold on
        plot(Vvs_atr_EOFF,lamdas_i_EOFF,syms(1),'Color','Black', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'Black', 'LineWidth',0.7, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
        ylabel("$\lambda_i [-]$",'FontSize', 20);
    hold all
    yyaxis right
        plot(Vvs_atr_MSB,Omegas_i_MSB,syms(6),'Color','Red', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'Red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(Vvs_atr_EOFF,Omegas_i_EOFF,syms(5),'Color','Red', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'Red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k')%,'FontSize', 13)
        ylabel("$\Omega_a [rad/s]$",'FontSize', 20);
    xlabel("$V_V$ [m/s]",'FontSize', 20);
    xlim([Vvs_atr_MSB(end) Vvs_atr_MSB(1)])
    grid minor
    legend("@mainShaftBroken","@EngineOffTransmissionOn",'Location','best','NumColumns',1, 'FontSize',18);
    saveas(gcf,"Graficas/lambdaivsVv_ATR",'epsc');

figure(10)
    yyaxis left
        plot(Vvs_atr_MSB,theta0s_i_MSB*(180/pi),syms(2), 'Color','red', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(Vvs_atr_EOFF,theta0s_i_EOFF*(180/pi),syms(1),'Color','red','MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
        ylabel("$\theta_0 [\mathrm{^o}]$",'FontSize', 20);
    hold all
    yyaxis right
        plot(Vvs_atr_MSB,thetaTs_i_MSB*(180/pi),syms(4),'Color','blue', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(Vvs_atr_EOFF,thetaTs_i_EOFF*(180/pi),syms(3),'Color','blue', 'MarkerIndices',5:5:length(Vvs_atr_MSB)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k')%,'FontSize', 13)
        ylabel("$\theta_T [\mathrm{^o}]$",'FontSize', 20);
    xlabel("$V_V$ [m/s]",'FontSize', 20);
    xlim([Vvs_atr_MSB(end) Vvs_atr_MSB(1)])
    %legend("$\theta_0$ (MSB)","$\theta_0$ (EOff/TOn)","$\theta_T$ (MSB)","$\theta_T$ (EOff/TOn)",'Location','west', 'NumColumns',1, 'FontSize',20);
    set(gcf, 'Color',[1 1 1]);
    grid minor
    hold off
    saveas(gcf,"Graficas/thetavsVv_ATR",'epsc');

%%
figure() % Darle una vuelta a esto
plot(Vvs_atr_MSB,CMys_0(:,1),syms(1),'Color','cyan','LineWidth',1); hold on
plot(Vvs_atr_EOFF,CMys_0(:,2),syms(1),'Color','cyan','LineWidth',1); hold on

plot(Vvs_atr_MSB,CMys_T(:,1),syms(1),'Color','magenta','LineWidth',1); hold on
plot(Vvs_atr_EOFF,CMys_T(:,2),syms(1),'Color','magenta','LineWidth',1); hold on


%% 5)  Análisis de la condición de vuelo equilibrado a nivel

close all;
clear,clc
disp("5)  Análisis de la condición de vuelo equilibrado a nivel");

% Se debe analizar una condición de vuelo equilibrado a nivel para velocidades V ∈ [0, 70] m/s.
% Las situaciones a analizar corresponden a ángulos de resbalamiento βf0 ∈ [−5º,0º,5º]. Analice
% cuidadosamente la forma de definir la condición de vuelo.

% 1. Representar las siguientes variables: ángulo de balanceo Φ, ángulo de cabeceo, Θ, parámetro de velocidad inducida del rotor principal, λi
% , parámetro de velocidad inducida del
% rotor antipar, λia, colectivo del rotor principal, θ0, paso cíclico lateral θ1C, paso cíclico
% longitudinal, θ1S, colectivo del rotor antipar, θT , ángulo de conicidad, β0, ángulo de batimiento longitudinal, β1C, ángulo de batimiento lateral, β1S, y potencia necesaria, PM, en
% función de la velocidad de vuelo. Comente las dificultades o particularidades del proceso
% de correcta definición de la condición de vuelo.
% 2. Represente la relación vi[T/(2ρS)]−0.5 función de V [T/(2ρS)]−0.5
% ,establezca las oportunas conclusiones sobre esta representación.
% 3. Determine las velocidades de máximo alcance y máxima autonomía para cada condición de resbalamiento analizada.
% 4. Represente las relaciones Pra(Prp)^−1 = f(V) para cada condición de resbalamiento.

h = 0;

[heRef, ndHeRef, atm, options] = Utils.createBo105(h);
muWT = [0; 0; 0];

VHs = linspace(0.001,70,36);
ndVHs = VHs/(heRef.mainRotor.R*heRef.mainRotor.Omega);
betaf0s = [-5,0,5]*(pi/180);

FC            = {'VOR',ndVHs,...
                 'betaf0',betaf0s,...
                 'wTOR',0,...
                 'cs',0,...
                 'vTOR',0};

disp("Calculando vuelo en avance.....");

tic
ndTrimState = getNdHeTrimState(ndHeRef,muWT,FC,options);
dimTrimState = ndHeTrimState2HeTrimState(ndTrimState,heRef,atm,h);
toc

Phis = zeros(length(VHs),length(betaf0s));
Thetas = zeros(length(VHs),length(betaf0s));

theta0s = zeros(length(VHs),length(betaf0s)); lambdais = zeros(length(VHs),length(betaf0s));
theta1Cs = zeros(length(VHs),length(betaf0s)); lambdaias = zeros(length(VHs),length(betaf0s));
theta1Ss = zeros(length(VHs),length(betaf0s)); PMs = zeros(length(VHs),length(betaf0s));
theta1Ts = zeros(length(VHs),length(betaf0s));

beta0s = zeros(length(VHs),length(betaf0s));
beta1Cs = zeros(length(VHs),length(betaf0s));
beta1Ss = zeros(length(VHs),length(betaf0s));

v_i_vio = zeros(length(VHs),length(betaf0s));
Vh_vio = zeros(length(VHs),length(betaf0s));

Prp_Pra = zeros(length(VHs),length(betaf0s));

for i = 1:length(VHs)
    for j = 1:length(betaf0s)
        
        Phis(i,j) = ndTrimState.solution.Phi(i,j);
        Thetas(i,j) = ndTrimState.solution.Theta(i,j);

        theta0s(i,j) = ndTrimState.solution.theta0(i,j); lambdais(i,j) = ndTrimState.solution.lambda0(i,j); 
        theta1Cs(i,j) = ndTrimState.solution.theta1C(i,j); lambdaias(i,j) = ndTrimState.solution.lambda0tr(i,j); 
        theta1Ss(i,j) = ndTrimState.solution.theta1S(i,j); PMs(i,j) = dimTrimState.Pow.PM(i,j); 
        theta1Ts(i,j) = ndTrimState.solution.theta0tr(i,j);

        beta0s(i,j) = ndTrimState.solution.beta0(i,j);
        beta1Cs(i,j) = ndTrimState.solution.beta1C(i,j);
        beta1Ss(i,j) = ndTrimState.solution.beta1S(i,j);

        v_io = (dimTrimState.solution.T0(i,j)/(atm.density(0)*2*pi*(heRef.mainRotor.R^2)))^0.5;
        v_i_vio(i,j) = (ndTrimState.solution.lambda0(i,j)*heRef.mainRotor.R*heRef.mainRotor.Omega)/v_io;

        Vh_vio(i,j) = VHs(i)/v_io;

        Prp_Pra(i,j) = dimTrimState.Pow.Pmr(i,j)/dimTrimState.Pow.Ptr(i,j);
        
    end

end
%%
syms = ["--","-","-s","--.","-*","--^"];

LW = 1.5;

figure(11)
    j = zeros(3,1);
    j(1) = plot(NaN,syms(1),'Color','black','MarkerFaceColor', 'black'); hold on
    j(2) = plot(NaN,syms(2),'Color','black','MarkerFaceColor', 'black'); hold on
    j(3) = plot(NaN,syms(3),'Color','black','MarkerFaceColor', 'black'); hold on
    
    h = zeros(2,1);
    plot(VHs,Phis(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(1) = plot(VHs,Phis(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Phis(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,1)*(180/pi),syms(1),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,Thetas(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    legend(h,"$\Phi$","$\Theta$",'Location','northwest','FontSize',20,'Orientation','horizontal'); 
    ylabel("$\Theta, \Phi [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    a=axes('position',get(gca,'position'),'visible','off');
    legend(a,j,"$\beta_{f0} = -5 \mathrm{^o}$","$\beta_{f0} = 0 \mathrm{^o}$","$\beta_{f0} = +5 \mathrm{^o}$",'NumColumns',1, 'Location','southwest','FontSize',20);
    
    saveas(gcf,"Graficas/ThetaPhivsVH_AV",'eps2c');

figure(13)
    plot(VHs,theta0s(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(1) = plot(VHs,theta0s(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot(VHs,theta1Ts(:,1)*(180/pi),syms(1),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,theta1Ts(:,2)*(180/pi),syms(2),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,3)*(180/pi),syms(3),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on

    legend(h,"$\theta_0$","$\theta_T$",'Location','southwest','FontSize',20);
    ylabel("$\theta_0, \theta_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetas0vsVH_AV",'eps2c');

figure(14)
    plot(VHs,theta1Cs(:,1)*(180/pi),syms(1),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(1) = plot(VHs,theta1Cs(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Cs(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot(VHs,theta1Ss(:,1)*(180/pi),syms(1),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,theta1Ss(:,2)*(180/pi),syms(2),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ss(:,3)*(180/pi),syms(3),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on

    legend(h,"$\theta_{1C}$","$\theta_{1S}$",'Location','southwest','FontSize',20);
    ylabel("$\theta_{1C},\theta_{1S} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetasvsVH_AV",'eps2c');

figure(15)
    plot(VHs,lambdais(:,1),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(1) = plot(VHs,lambdais(:,2),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,lambdais(:,3),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot(VHs,lambdaias(:,1),syms(1),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,lambdaias(:,2),syms(2),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,lambdaias(:,3),syms(3),'Color','magenta','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'magenta', 'LineWidth',LW, 'MarkerSize', 4); hold on

    legend(h,"$\lambda_i$","$\lambda_{ia}$",'Location','best','FontSize',20);
    ylabel("$\lambda_i,\lambda_{ia} [-]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/lambdasvsVH_AV",'eps2c');

figure(16)
    h = zeros(3,1);
    plot(VHs,beta0s(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(1) = plot(VHs,beta0s(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,beta0s(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot(VHs,beta1Cs(:,1)*(180/pi),syms(1),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,beta1Cs(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,beta1Cs(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on

    plot(VHs,beta1Ss(:,1)*(180/pi),syms(1),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(3) = plot(VHs,beta1Ss(:,2)*(180/pi),syms(2),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,beta1Ss(:,3)*(180/pi),syms(3),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on

    legend(h,"$\beta_0$","$\beta_{1C}$","$\beta_{1S}$",'Location','best','FontSize',20,'Orientation','horizontal');
    ylabel("$\beta_0,\beta_{1C},\beta_{1S} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/betasvsVH_AV",'eps2c');

figure(17)
    plot(VHs,PMs(:,1)*0.001,syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,PMs(:,2)*0.001,syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,PMs(:,3)*0.001,syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    [P_max_alcance, P_max_autonm,V_max_alcance, V_max_autonm] = Utils.getHePerformance(dimTrimState,VHs,true);

    scatter(V_max_alcance(2),P_max_alcance(2)*0.001,'MarkerEdgeColor','black','LineWidth',2); hold on
    scatter(V_max_autonm(2),P_max_autonm(2)*0.001,'MarkerEdgeColor','black','LineWidth',2); hold on

    annotation('textbox',[.56 .22 .4 .27],'EdgeColor','none','String','$C$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.82 .22 .4 .33],'EdgeColor','none','String','$D$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on

    plot([0, V_max_alcance(2)],[0,P_max_alcance(2)*0.001],'--','Color','black','LineWidth',0.7); hold on
    ylabel("$P_M$ [kW]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/PMsvsVH_AV",'eps2c');


figure(18)
    plot(Vh_vio(:,1),-v_i_vio(:,1),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(Vh_vio(:,2),-v_i_vio(:,2),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(Vh_vio(:,3),-v_i_vio(:,3),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    ylabel("$v_i/v_io [-]$",'FontSize',20);
    xlabel("$V_V/v_io [-]$",'FontSize',20);
    legend("$\beta_{f0} = -5 \mathrm{^o}$","$\beta_{f0} = 0 \mathrm{^o}$","$\beta_{f0} = +5 \mathrm{^o}$",'NumColumns',1, 'Location','northeast','FontSize',20);
    grid minor
    saveas(gcf,"Graficas/viosvsVH_AV",'eps2c');

figure(19)
    plot(VHs,Prp_Pra(:,1),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Prp_Pra(:,2),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Prp_Pra(:,3),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    ylabel("$P^{rp}/P^{ra} [-]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/PraPrpsvsVH_AV",'eps2c');

%% 6) Análisis de la condición de vuelo equilibrado inclinado.

close all;
clear,clc
disp("6) Análisis de la condición de vuelo equilibrado inclinado");

% Se debe analizar una condición de vuelo equilibrado inclinado para velocidades V ∈ [0, 70] m/s
% y ángulos de inclinación de la trayectoria γT = [−60º, −30º, 0º, 5º,10º]. Las situaciones a analizar corresponden a ángulo de resbalamiento nulo, βf0 = 0. Analice cuidadosamente la forma
% de definir la condición de vuelo.

% 1. Representar las siguientes variables: ángulo de cabeceo, Θ, colectivo del rotor principal,
% θ0, paso cíclico lateral θ1C, paso cíclico longitudinal, θ1S, colectivo del rotor antipar, y
% potencia necesaria, PM, en función de la velocidad de vuelo. Comente las dificultades o
% particularidades del proceso de correcta definición de la condición de vuelo.
% 2. Analice la bondad de la hipótesis de fuerzas resultantes del rotor principal perpendiculares
% al plano de puntas.

h = 0;

[heRef, ndHeRef, atm, options] = Utils.createBo105(h);
muWT = [0; 0; 0];

VHs = linspace(0,70,36);
ndVHs = VHs/(heRef.mainRotor.R*heRef.mainRotor.Omega);

gammaTs = [-60 -30 0 5 10]*(pi/180);

FC            = {'VOR',ndVHs,...
                 'gammaT',gammaTs,...
                 'betaf0',0,...
                 'cs',0,...
                 'vTOR',0};

disp("Calculando vuelo en avance.....");

tic
ndTrimState = getNdHeTrimState(ndHeRef,muWT,FC,options);
dimTrimState = ndHeTrimState2HeTrimState(ndTrimState,heRef,atm,h);
toc

Thetas = zeros(length(VHs),length(gammaTs));

theta0s = zeros(length(VHs),length(gammaTs)); 
theta1Cs = zeros(length(VHs),length(gammaTs)); 
theta1Ss = zeros(length(VHs),length(gammaTs)); 
theta1Ts = zeros(length(VHs),length(gammaTs));

PMs = zeros(length(VHs),length(gammaTs));

for i = 1:length(VHs)
    for j = 1:length(gammaTs)
        
        Thetas(i,j) = ndTrimState.solution.Theta(i,j);

        theta0s(i,j) = ndTrimState.solution.theta0(i,j); 
        theta1Cs(i,j) = ndTrimState.solution.theta1C(i,j); 
        theta1Ss(i,j) = ndTrimState.solution.theta1S(i,j);
        theta1Ts(i,j) = ndTrimState.solution.theta0tr(i,j);

        PMs(i,j) = dimTrimState.Pow.PM(i,j); 
    end
end
%%
LW = 1.5;
syms = ["--","-^","-","--p","-o",""];

figure()
    plot(VHs,theta0s(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,4)*(180/pi),syms(4),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,5)*(180/pi),syms(5),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    ylabel("$\theta_0 [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetas0vsVH_AV_gammaT",'eps2c');

figure()
    plot(VHs,theta1Ts(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,4)*(180/pi),syms(4),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,5)*(180/pi),syms(5),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    ylabel("$\theta_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetasTsvsVH_AV_gammaT",'eps2c');

figure()
    plot(VHs,theta1Cs(:,1)*(180/pi),syms(1),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Cs(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Cs(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Cs(:,4)*(180/pi),syms(4),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Cs(:,5)*(180/pi),syms(5),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    ylabel("$\theta_{1C} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetas1SsvsVH_AV_gammaT",'eps2c');

figure()
    plot(VHs,theta1Ss(:,1)*(180/pi),syms(1),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ss(:,2)*(180/pi),syms(2),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ss(:,3)*(180/pi),syms(3),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ss(:,4)*(180/pi),syms(4),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ss(:,5)*(180/pi),syms(5),'Color','blue','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    ylabel("$\theta_{1S} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetas1CsvsVH_AV_gammaT",'eps2c');

figure()
    plot(VHs,Thetas(:,1)*(180/pi),syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,2)*(180/pi),syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,3)*(180/pi),syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,4)*(180/pi),syms(4),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Thetas(:,5)*(180/pi),syms(5),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    ylabel("$\Theta [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/ThetasvsVH_AV_gammaT",'eps2c');

figure()
    h = zeros(5,1);
    h(1) = plot(VHs,PMs(:,1)*0.001,syms(1),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(2) = plot(VHs,PMs(:,2)*0.001,syms(2),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(3) = plot(VHs,PMs(:,3)*0.001,syms(3),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(5) = plot(VHs,PMs(:,4)*0.001,syms(4),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    h(4) = plot(VHs,PMs(:,5)*0.001,syms(5),'Color','Black','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    
    legend(h,"$\gamma_T=-60\mathrm{^o}$","$\gamma_T=-30\mathrm{^o}$","$\gamma_T=0\mathrm{^o}$","$\gamma_T=5\mathrm{^o}$","$\gamma_T=10\mathrm{^o}$",'Location','northwest','NumColumns',2,'FontSize',20);
    
    ylabel("$P_M$ [kW]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/PMsvsVH_AV_gammaT",'eps2c');

%% 7) Análisis de la condición de vuelo equilibrado en autorrotación con avance.

close all;
clear,clc
disp("7) Análisis de la condición de vuelo equilibrado en autorrotación con avance.");

% Debe analizar una condición de vuelo equilibrado en autorrotación con avance para velocidades de avance VH ∈ [0, 70] m/s. Analice cuidadosamente la forma de definir la condición de
% vuelo e identifique las dificultades encontradas.

% 1. Representar las siguientes variables: colectivo del rotor principal, θ0, paso cíclico lateral
% θ1C, paso cíclico longitudinal, θ1S, colectivo del rotor antipar, θT y potencia necesaria, PM, en función de la velocidad de vuelo, VH.
% 2. Determine la función VV = f(VH) y γT = g(VH).
% 3. Represente la dependencia paramétrica de VV = f(VH, Ωa) y γT = f(VH, Ωa) donde Ωa
% es la velocidad de giro del rotor en la fase de autorrotación estacionaria

h = 0;

[heRef, ndHeRef, atm, options] = Utils.createBo105(h);
muWT = [0; 0; 0];

ndVv = 20/(heRef.mainRotor.R*heRef.mainRotor.Omega);

FC_prem         = {'VOR', 1e-5,...
                 'Psi',0,...
                 'gammaT',0,...
                 'cs',0,...
                 'vTOR',0};

Vv_vio = 1.732;
disp( "-----------------------------------------------------");
disp("Calculando punto fijo.....");
tic
ndTrimState_prem = getNdHeTrimState(ndHeRef,muWT,FC_prem,options);
dimTrimState_prem = ndHeTrimState2HeTrimState(ndTrimState_prem,heRef,atm,h);
toc

v_io = (dimTrimState_prem.solution.T0/(atm.density(0)*2*pi*(heRef.mainRotor.R)^2))^0.5;

ndVv = (Vv_vio/v_io);

%[heRef, ndHeRef, atm, options] = Utils.createBo105(h,false,ndTrimState_prem.solution);

%Vv = linspace(0,70,10);
%ndVv = Vv/(heRef.mainRotor.R*heRef.mainRotor.Omega);
FC_0            = {'VOR', ndVv,...
                 'Psi',0,...
                 'gammaT',-pi/2,...
                 'cs',0,...
                 'vTOR',0};

disp("Calculando autorrotacion vertical.....");
tic
ndTrimState_0 = getNdHeTrimState(ndHeRef,muWT,FC_0,options);
dimTrimState_0 = ndHeTrimState2HeTrimState(ndTrimState_0,heRef,atm,h);
toc

disp( "-----------------------------------------------------");

% AUTORROTACIÃ“N (SOLO @MAINSHAFTBROKEN PERO TENGO MIS DUDAS SI ES MEJOR CON
% @ENGINEOFF, SALEN COSAS RARAS)

[heRef, ndHeRef, atm, options] = Utils.createBo105(h,true,ndTrimState_0.solution);

options.engineState = @mainShaftBroken;
% options.engineState = @EngineOffTransmissionOn;

VHs = linspace(0.75,70,36);
ndVHs = VHs/(heRef.mainRotor.R*heRef.mainRotor.Omega);

o_on = [0.85,1,1.15];

FC_atr         = {'uTOR', ndVHs,...
                 'omega',o_on,...
                 'Psi',0,...
                 'cs',0,...
                 'vTOR',0};

disp("Calculado condicion de autorrotacion en avance......");
tic
ndTrimState_atr = getNdHeTrimState(ndHeRef,muWT,FC_atr,options);
dimTrimState_atr = ndHeTrimState2HeTrimState(ndTrimState_atr,heRef,atm,h);
toc

theta0s = zeros(length(VHs),length(o_on)); 
theta1Cs = zeros(length(VHs),length(o_on)); 
theta1Ss = zeros(length(VHs),length(o_on)); 
theta1Ts = zeros(length(VHs),length(o_on));

PMs = zeros(length(VHs),length(o_on));

Vvs = zeros(length(VHs),length(o_on));
gammaTs  = zeros(length(VHs),length(o_on));

for i = 1:length(VHs)
    for j = 1:length(o_on)

        theta0s(i,j) = ndTrimState_atr.solution.theta0(i,j); 
        theta1Cs(i,j) = ndTrimState_atr.solution.theta1C(i,j); 
        theta1Ss(i,j) = ndTrimState_atr.solution.theta1S(i,j);
        theta1Ts(i,j) = ndTrimState_atr.solution.theta0tr(i,j);

        PMs(i,j) = dimTrimState_atr.Pow.PM(i,j); 

        Vvs(i,j) = -dimTrimState_atr.solution.wT(i,j);
        gammaTs(i,j) = ndTrimState_atr.solution.gammaT(i,j);

    end
end


LW = 1.5;

syms = ["--","-","-s","--.","-*","--^"];

figure()
    plot(VHs,theta0s(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta0s(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    ylabel("$\theta_0 [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetas0svsVH_AV_ATR",'eps2c');

figure()
    plot(VHs,theta1Ts(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,theta1Ts(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    legend("$\Omega_a/\Omega_N=0.85$","$\Omega_a/\Omega_N=1$","$\Omega_a/\Omega_N=1.15$",'FontSize',20,'Location','best');
    ylabel("$\theta_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/thetasTsvsVH_AV_ATR",'eps2c');


figure()
    yyaxis left
        plot(VHs,theta1Cs(:,1)*(180/pi),syms(1), 'Color','red', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        h(1)=plot(VHs,theta1Cs(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(VHs,theta1Cs(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
        ylabel("$\theta_{1C} [\mathrm{^o}]$",'FontSize',20);
    hold all
    yyaxis right
        plot(VHs,theta1Ss(:,1)*(180/pi),syms(1),'Color','blue', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        h(2)=plot(VHs,theta1Ss(:,2)*(180/pi),syms(2),'Color','blue', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(VHs,theta1Ss(:,3)*(180/pi),syms(3),'Color','blue', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k')%,'FontSize', 13)
        ylabel("$\theta_{1S} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs(1) VHs(end)])
    legend(h,"$\theta_{1C}$","$\theta_{1S}$",'NumColumns',2, 'FontSize',20,'Location','south');
    set(gcf, 'Color',[1 1 1]);
    grid minor
    hold off
    saveas(gcf,"Graficas/thetas1SvsVH_AV_ATR",'eps2c');




figure()
    plot(VHs,PMs(:,1)*0.001,syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,PMs(:,2)*0.001,syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,PMs(:,3)*0.001,syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    ylabel("$P_M$ [kW]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/PMsvsVH_AV_ATR",'eps2c');


[Vv_min, iVvmin] = max(Vvs(:,2)); 
Vv_Vh = zeros(1,length(VHs));
VORs = zeros(1,length(VHs));
for i = 1:length(Vv_Vh)

    Vv_Vh(i) = Vvs(i,2)/VHs(i);
    VORs(i) = sqrt(Vvs(i,2)^2 + VHs(i)^2);
end

[VvgammaT_min, igammaT_min] = max((Vv_Vh));
[VORs,iVORmin] = min(VORs);

figure()
    plot(VHs,Vvs(:,1),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Vvs(:,2),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,Vvs(:,3),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    xline(VHs(iVvmin),"-.",'Color','black','LineWidth',0.7); hold on
    xline(VHs(igammaT_min),"-.","Color","black","LineWidth",0.7); hold on
    xline(VHs(iVORmin),"-.","Color","black","LineWidth",0.7); hold on
    scatter(VHs(iVvmin),Vv_min,"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(igammaT_min),Vvs(igammaT_min,2),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(iVORmin),Vvs(iVORmin,2),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    plot([0,VHs(igammaT_min)],[0, Vvs(igammaT_min,2)],"-.","Color","black",'LineWidth',0.7); hold on

    annotation('textbox',[.15 .18 .4 .2],'EdgeColor','none','String','$B$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.50 .4 .4 .3],'EdgeColor','none','String','$C$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.82 .3 .4 .3],'EdgeColor','none','String','$D$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on

    annotation('textbox',[.38 .5 .4 .4],'EdgeColor','none','String','$(\gamma_T)_{min}$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.35 .5 .4 .4],'EdgeColor','none','String','$\downarrow$','FontSize',30,'Linewidth',5,'Interpreter','latex'); hold on

    ylabel("$V_V$ [m/s]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/VVsvsVH_AV_ATR",'eps2c');

figure()
    plot(VHs,gammaTs(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,gammaTs(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs,gammaTs(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs)-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    xline(VHs(iVvmin),"-.",'Color','black','LineWidth',0.7); hold on
    xline(VHs(igammaT_min),"-.","Color","black","LineWidth",0.7); hold on
    xline(VHs(iVORmin),"-.","Color","black","LineWidth",0.7); hold on
    scatter(VHs(iVvmin),gammaTs(iVvmin,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(igammaT_min),gammaTs(igammaT_min,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(iVORmin),gammaTs(iVORmin,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on

    annotation('textbox',[.14 .22 .4 .2],'EdgeColor','none','String','$B$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.53 .4 .4 .4],'EdgeColor','none','String','$C$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.75 .43 .4 .4],'EdgeColor','none','String','$D$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    legend("$\Omega_a/\Omega_N=0.85$","$\Omega_a/\Omega_N=1$","$\Omega_a/\Omega_N=1.15$",'FontSize',20,'Location','best');
    ylabel("$\gamma_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    grid minor
    saveas(gcf,"Graficas/gammaTsvsVH_AV_ATR",'eps2c');

%% 8) Análisis paramétrico de la condición de vuelo equilibrado de avance a nivel.

close all;
clear,clc
disp("8) Análisis paramétrico de la condición de vuelo equilibrado de avance a nivel.");

% Se pretende establecer la influencia de dos parámetros relevantes del diseño de la aeronave
% como son la posición longitudinal del centro de gravedad xCG y la superficie del estabilizador
% vertical, Sev en la condición de vuelo de equilibrado de avance a nivel. Con la experiencia
% adquirida en los apartados anteriores, en concreto en el apartado 1.2.5, se propone identificar que
% 5 parámetros de los que definen la condición de vuelo a nivel se ven afectados por las variaciones
% de ambos parámetros de diseño. Observe que es fundamental establecer valores adecuados para
% el intervalo de velocidades de avance a analizar, el intervalo de variación de xCG y de Sev. tenga
% en cuenta que ha de elegir intervalos de variación significativos y realistas a la vez.

h = 0;

[heRef, ~, atm,options] = Utils.createBo105();
muWT = [0; 0; 0];

xcgs = [heRef.geometry.xcg - 1, heRef.geometry.xcg,heRef.geometry.xcg + 1];
Sws = [0.5,1,1.5]*heRef.verticalFin.S;

heRefs_Xcg = getParametricCellHe(heRef,'geometry.xcg',xcgs); 
heRefs_Sws = getParametricCellHe(heRef,'verticalFin.S',Sws); 

VHs = linspace(0.001,100,50);
ndVHs = VHs/(heRef.mainRotor.R*heRef.mainRotor.Omega);

FC            = {'VOR',ndVHs,...
                 'Psi',0,...
                 'wTOR',0,...
                 'cs',0,...
                 'vTOR',0};

thetas0_cg = zeros(length(xcgs),length(VHs)); betas1C_cg = zeros(length(xcgs),length(VHs));
thetas1C_cg = zeros(length(xcgs),length(VHs)); betas1S_cg = zeros(length(xcgs),length(VHs));
thetas1S_cg = zeros(length(xcgs),length(VHs)); Thetas_cg = zeros(length(xcgs),length(VHs));

Thetas_Sw = zeros(length(xcgs),length(VHs));
thetasT_Sw = zeros(length(xcgs),length(VHs));
lambdaias_Sw = zeros(length(xcgs),length(VHs));
prm_prps_Sw = zeros(length(xcgs),length(VHs));

CMty_mr = zeros(length(xcgs),length(VHs));
CMty_ht = zeros(length(xcgs),length(VHs));
CMty_f = zeros(length(xcgs),length(VHs));

for i = 1:length(xcgs)

        ndHeRef_cg = rigidHe2ndHe(heRefs_Xcg{i},atm,h);
        ndHeRef_Sw = rigidHe2ndHe(heRefs_Sws{i},atm,h);
        
        tic
        ndTrimState_cg = getNdHeTrimState(ndHeRef_cg,muWT,FC,options);
        ndTrimState_Sw = getNdHeTrimState(ndHeRef_Sw,muWT,FC,options);

        dimTrimState_cg = ndHeTrimState2HeTrimState(ndTrimState_cg,heRefs_Xcg{i},atm,h);
        dimTrimState_Sw = ndHeTrimState2HeTrimState(ndTrimState_Sw,heRefs_Sws{i},atm,h);
        toc

        for j = 1:length(VHs)
            
            thetas0_cg(i,j) =  ndTrimState_cg.solution.theta0(j); betas1C_cg(i,j) =  ndTrimState_cg.solution.beta1C(j);
            thetas1C_cg(i,j) =  ndTrimState_cg.solution.theta1C(j); betas1S_cg(i,j) =  ndTrimState_cg.solution.beta1S(j);
            thetas1S_cg(i,j) =  ndTrimState_cg.solution.theta1S(j); Thetas_cg(i,j) =  ndTrimState_cg.solution.Theta(j);

            Thetas_Sw(i,j) =  ndTrimState_Sw.solution.theta0(j);
            thetasT_Sw(i,j) =  ndTrimState_Sw.solution.theta0tr(j);
            lambdaias_Sw(i,j) =  ndTrimState_Sw.solution.lambda0tr(j);
            prm_prps_Sw(i,j) =  dimTrimState_Sw.Pow.Pmr(j)/dimTrimState_Sw.Pow.Ptr(j);
            
            CMty_mr(i,j) = ndTrimState_cg.actions.mainRotor.fuselage.CMty(j);
            CMty_ht(i,j) = ndTrimState_cg.actions.leftHTP.fuselage.CMty(j) + ndTrimState_cg.actions.rightHTP.fuselage.CMty(j);
            CMty_f(i,j) = ndTrimState_cg.actions.fuselage.fuselage.CMty(j);

        end
        
%         axds          = getaxds({'VOR'},{'$$V/(\Omega R)$$ [-]'},1);
% 
%         azdsACT       = getaxds({'CMtx' 'CMty' 'CMtz'},...
%                       {'$$C_{Mx}$$ [-]' '$$C_{My}$$ [-]' '$$C_{Mz}$$ [-]'}, ...
%                       [1 1 1 ...
%                        1 1 1]);
% 
%         plotActionsByElement(ndTrimState_cg.actions,axds,ndVHs,'defaultVars',azdsACT);

end
%%
LW = 1.5;

syms = ["--","-","-x","-.","-x","-o","-*"];

figure()
plot(VHs,thetas0_cg(1,:)*(180/pi),syms(1),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetas0_cg(2,:)*(180/pi),syms(2),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetas0_cg(3,:)*(180/pi),syms(3),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

ylabel("$\theta_{0}[\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/thetas0svsVH_XCG",'eps2c');

figure()
h = zeros(2,1);
plot(VHs,thetas1C_cg(1,:)*(180/pi),syms(1),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
h(1) = plot(VHs,thetas1C_cg(2,:)*(180/pi),syms(2),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetas1C_cg(3,:)*(180/pi),syms(3),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

plot(VHs,thetas1S_cg(1,:)*(180/pi),syms(1),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
h(2) = plot(VHs,thetas1S_cg(2,:)*(180/pi),syms(2),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetas1S_cg(3,:)*(180/pi),syms(3),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

legend(h,"$\theta_{1C}$","$\theta_{1S}$",'FontSize',20,'Location','southwest');
ylabel("$\theta_{1C},\theta_{1S} [\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/thetas1svsVH_XCG",'eps2c');

figure()
h = zeros(2,1);
plot(VHs,betas1C_cg(1,:)*(180/pi),syms(1),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
h(1) = plot(VHs,betas1C_cg(2,:)*(180/pi),syms(2),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,betas1C_cg(3,:)*(180/pi),syms(3),'Color','red','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

plot(VHs,betas1S_cg(1,:)*(180/pi),syms(1),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
h(2) = plot(VHs,betas1S_cg(2,:)*(180/pi),syms(2),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,betas1S_cg(3,:)*(180/pi),syms(3),'Color','blue','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

legend(h,"$\beta_{1C}$","$\beta_{1S}$",'FontSize',20,'Location','best','Orientation','horizontal');
ylabel("$\beta_{1C},\beta_{1S} [\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/betas1svsVH_XCG",'eps2c');

figure()
plot(VHs,Thetas_cg(1,:)*(180/pi),syms(1),'Color','black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,Thetas_cg(2,:)*(180/pi),syms(2),'Color','black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,Thetas_cg(3,:)*(180/pi),syms(3),'Color','black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

legend("$x_{cg} - 1$","$x_{cg}$","$x_{cg} + 1$",'FontSize',20,'Location','southwest');
ylabel("$\Theta [\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/ThetassvsVH_XCG",'eps2c');

figure()
h= zeros(3,1);

subplot(1,3,1);
plot(VHs,CMty_mr(1,:),syms(1),'Color','black','LineWidth',LW); hold on
plot(VHs,CMty_ht(1,:),syms(2),'Color','red','LineWidth',LW); hold on
plot(VHs,CMty_f(1,:),syms(4),'Color','blue','LineWidth',LW); hold on

annotation('textbox',[.18 .1 .4 .15],'EdgeColor','none','String','$x_{cg} - 1$','FontSize',14,'Linewidth',5,'Interpreter','latex'); hold on

ylim([-4 4]*10^(-4));
ylabel("$CMt_y [-]$",'FontSize',14);
xlabel("$V_H$ [m/s]",'FontSize',14);
grid minor

subplot(1,3,2);
h(1) = plot(VHs,CMty_mr(2,:),syms(1),'Color','Black','LineWidth',LW); hold on
h(2) = plot(VHs,CMty_ht(2,:),syms(2),'Color','red','LineWidth',LW); hold on
h(3) = plot(VHs,CMty_f(2,:),syms(4),'Color','blue','LineWidth',LW); hold on

annotation('textbox',[.48 .1 .4 .15],'EdgeColor','none','String','$x_{cg}$','FontSize',14,'Linewidth',5,'Interpreter','latex'); hold on

ylim([-4 4]*10^(-4));
xlabel("$V_H$ [m/s]",'FontSize',14);
grid minor

subplot(1,3,3);
plot(VHs,CMty_mr(3,:),syms(1),'Color','black','LineWidth',LW); hold on
plot(VHs,CMty_ht(3,:),syms(2),'Color','red','LineWidth',LW); hold on
plot(VHs,CMty_f(3,:),syms(4),'Color','blue','LineWidth',LW); hold on

annotation('textbox',[.75 .1 .4 .15],'EdgeColor','none','String','$x_{cg} + 1$','FontSize',14,'Linewidth',5,'Interpreter','latex'); hold on

legend(h,"$rp$","$eh$","$f$",'FontSize',14,'Location','best');
ylim([-4 4]*10^(-4));
xlabel("$V_H$ [m/s]",'FontSize',14);
grid minor

saveas(gcf,"Graficas/CmtyvsVH_XCG",'eps2c');

%SW
figure()
plot(VHs,Thetas_Sw(1,:)*(180/pi),syms(1),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,Thetas_Sw(2,:)*(180/pi),syms(2),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,Thetas_Sw(3,:)*(180/pi),syms(3),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

legend("$0.5S_w$","$S_w$","$1.5S_w$",'Location','northwest','FontSize',20);
ylabel("$\theta_0[\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/ThetasvsVH_SW",'eps2c');

figure()
plot(VHs,thetasT_Sw(1,:)*(180/pi),syms(1),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetasT_Sw(2,:)*(180/pi),syms(2),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,thetasT_Sw(3,:)*(180/pi),syms(3),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

ylabel("$\theta_T [\mathrm{^o}]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/thetasTsvsVH_SW",'eps2c');

figure()
plot(VHs,lambdaias_Sw(1,:),syms(1),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,lambdaias_Sw(2,:),syms(2),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,lambdaias_Sw(3,:),syms(3),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

ylabel("$\lambda_{ia} [-]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/lambdasvsVH_SW",'eps2c');

figure()
plot(VHs,prm_prps_Sw(1,:),syms(1),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,prm_prps_Sw(2,:),syms(2),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on
plot(VHs,prm_prps_Sw(3,:),syms(3),'Color','Black','LineWidth',LW,'MarkerIndices',5:5:length(VHs)-1,'MarkerSize',4); hold on

ylabel("$P_{rp}/P_{ra} [-]$",'FontSize',20);
xlabel("$V_H$ [m/s]",'FontSize',20);
grid minor
saveas(gcf,"Graficas/prmprpsvsVH_SW",'eps2c');
