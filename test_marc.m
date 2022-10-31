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

disp( "-----------------------------------------------------");

% AUTORROTACION

[heRef, ndHeRef, atm, options] = Utils.createBo105(h,true,ndTrimState_prem.solution);

options.engineState = @mainShaftBroken;
% options.engineState = @EngineOffTransmissionOn;

o_on = [0.85,1,1.15];
VHs_i = [1e-4 linspace(2,70,35)];

ndVHs = VHs_i/(heRef.mainRotor.R*heRef.mainRotor.Omega*o_on(1));



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

theta0s = zeros(length(VHs_i),length(o_on)); 
theta1Cs = zeros(length(VHs_i),length(o_on)); 
theta1Ss = zeros(length(VHs_i),length(o_on)); 
theta1Ts = zeros(length(VHs_i),length(o_on));

PMs = zeros(length(VHs_i),length(o_on));

Vvs = zeros(length(VHs_i),length(o_on));
gammaTs  = zeros(length(VHs_i),length(o_on));

for i = 1:length(VHs_i)
    for j = 1:length(o_on)
        
        theta0s(i,j) = ndTrimState_atr.solution.theta0(i,j); 
        theta1Cs(i,j) = ndTrimState_atr.solution.theta1C(i,j); 
        theta1Ss(i,j) = ndTrimState_atr.solution.theta1S(i,j);
        theta1Ts(i,j) = ndTrimState_atr.solution.theta0tr(i,j);

        PMs(i,j) = dimTrimState_atr.Pow.PM(i,j); 

        VHs(i,j) = dimTrimState_atr.solution.uT(i,j);
        Vvs(i,j) = -dimTrimState_atr.solution.wT(i,j);
        gammaTs(i,j) = ndTrimState_atr.solution.gammaT(i,j);
        for k=1:15
            if gammaTs(i,j) > pi/2*(k-1) && gammaTs(i,j) < pi/2*k
                gammaTs(i,j) = gammaTs(i,j)-pi/2*k;
            end
        end
    end
end


LW = 1.5;

syms = ["--","-","-s","--.","-*","--^"];

figure()
    plot(VHs(:,1),theta0s(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,2),theta0s(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,3),theta0s(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
    ylabel("$\theta_0 [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    grid minor
    saveas(gcf,"Graficas/thetas0svsVH_AV_ATR",'eps2c');

figure()
    plot(VHs(:,1),theta1Ts(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,2),theta1Ts(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,3),theta1Ts(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
    legend("$\Omega_a/\Omega_N=0.85$","$\Omega_a/\Omega_N=1$","$\Omega_a/\Omega_N=1.15$",'FontSize',20,'Location','best');
    ylabel("$\theta_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    grid minor
    saveas(gcf,"Graficas/thetasTsvsVH_AV_ATR",'eps2c');


figure()
    yyaxis left
        plot(VHs(:,1),theta1Cs(:,1)*(180/pi),syms(1), 'Color','red', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        h(1)=plot(VHs(:,2),theta1Cs(:,2)*(180/pi),syms(2),'Color','red','MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(VHs(:,3),theta1Cs(:,3)*(180/pi),syms(3),'Color','red','MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor', 'red', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
        ylabel("$\theta_{1C} [\mathrm{^o}]$",'FontSize',20);
    hold all
    yyaxis right
        plot(VHs(:,1),theta1Ss(:,1)*(180/pi),syms(1),'Color','blue', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        h(2)=plot(VHs(:,2),theta1Ss(:,2)*(180/pi),syms(2),'Color','blue', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        plot(VHs(:,3),theta1Ss(:,3)*(180/pi),syms(3),'Color','blue', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor', 'blue', 'LineWidth',LW, 'MarkerSize', 4); hold on
        set(gca, 'XColor','k', 'YColor','k')%,'FontSize', 13)
        ylabel("$\theta_{1S} [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    legend(h,"$\theta_{1C}$","$\theta_{1S}$",'NumColumns',2, 'FontSize',20,'Location','south');
    set(gcf, 'Color',[1 1 1]);
    grid minor
    hold off
    saveas(gcf,"Graficas/thetas1SvsVH_AV_ATR",'eps2c');




figure()
    plot(VHs(:,1),PMs(:,1)*0.001,syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,2),PMs(:,2)*0.001,syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,3),PMs(:,3)*0.001,syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
    ylabel("$P_M$ [kW]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    grid minor
    saveas(gcf,"Graficas/PMsvsVH_AV_ATR",'eps2c');
%%

[Vv_min, iVvmin] = max(Vvs(:,2)); 
Vv_Vh = zeros(1,length(VHs(:,2)));
VORs = zeros(1,length(VHs(:,2)));
for i = 1:length(Vv_Vh)

    Vv_Vh(i) = Vvs(i,2)/VHs(i);
    VORs(i) = sqrt(Vvs(i,2)^2 + VHs(i)^2);
end

[VvgammaT_min, igammaT_min] = max((Vv_Vh));
[VORs,iVORmin] = min(VORs);

figure()
    plot(VHs(:,1),Vvs(:,1),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,2),Vvs(:,2),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,3),Vvs(:,3),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    xline(VHs(iVvmin,2),"-.",'Color','black','LineWidth',0.7); hold on
    xline(VHs(igammaT_min,2),"-.","Color","black","LineWidth",0.7); hold on
    xline(VHs(iVORmin,2),"-.","Color","black","LineWidth",0.7); hold on
    scatter(VHs(iVvmin,2),Vv_min,"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(igammaT_min,2),Vvs(igammaT_min,2),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(iVORmin,2),Vvs(iVORmin,2),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    plot([0,VHs(igammaT_min,2)],[0, Vvs(igammaT_min,2)],"-.","Color","black",'LineWidth',0.7); hold on

    annotation('textbox',[.18 .22 .4 .2],'EdgeColor','none','String','$B$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.50 .39 .4 .3],'EdgeColor','none','String','$C$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.82 .3 .4 .3],'EdgeColor','none','String','$D$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on

    annotation('textbox',[.38 .5 .4 .4],'EdgeColor','none','String','$|(\gamma_T)|_{min}$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.35 .5 .4 .4],'EdgeColor','none','String','$\downarrow$','FontSize',30,'Linewidth',5,'Interpreter','latex'); hold on

    set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
    ylabel("$V_V$ [m/s]",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    grid minor
    saveas(gcf,"Graficas/VVsvsVH_AV_ATR",'eps2c');

figure()
    plot(VHs(:,1),gammaTs(:,1)*(180/pi),syms(1), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,1))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,2),gammaTs(:,2)*(180/pi),syms(2), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,2))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on
    plot(VHs(:,3),gammaTs(:,3)*(180/pi),syms(3), 'Color','Black', 'MarkerIndices',5:5:length(VHs(:,3))-1, 'MarkerFaceColor','Black', 'LineWidth',LW, 'MarkerSize', 4); hold on

    xline(VHs(iVvmin,2),"-.",'Color','black','LineWidth',0.7); hold on
    xline(VHs(igammaT_min,2),"-.","Color","black","LineWidth",0.7); hold on
    xline(VHs(iVORmin,2),"-.","Color","black","LineWidth",0.7); hold on
    scatter(VHs(iVvmin,2),gammaTs(iVvmin,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(igammaT_min,2),gammaTs(igammaT_min,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on
    scatter(VHs(iVORmin,2),gammaTs(iVORmin,2)*(180/pi),"MarkerEdgeColor",'black',"LineWidth",2); hold on

    annotation('textbox',[.16 .3 .4 .22],'EdgeColor','none','String','$B$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.53 .4 .4 .4],'EdgeColor','none','String','$C$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    annotation('textbox',[.75 .43 .4 .4],'EdgeColor','none','String','$D$','FontSize',20,'Linewidth',5,'Interpreter','latex'); hold on
    
    set(gca, 'XColor','k', 'YColor','k','FontSize', 13)
    legend("$\Omega_a/\Omega_N=0.85$","$\Omega_a/\Omega_N=1$","$\Omega_a/\Omega_N=1.15$",'FontSize',20,'Location','southeast');
    ylabel("$\gamma_T [\mathrm{^o}]$",'FontSize',20);
    xlabel("$V_H$ [m/s]",'FontSize',20);
    xlim([VHs_i(1) VHs_i(end)])
    grid minor
    saveas(gcf,"Graficas/gammaTsvsVH_AV_ATR",'eps2c');