classdef Utils
    
    % -- Contains useful functions (converting units, etc) --
    % Also contains ISA functions, scaling functions,TF manipulation utils
    % and sensor/actuator modelling TFs.
    
    methods (Static)    
        %% ---- UNIT CONVERSION ----
        
        function m_value = ft_to_m(ft_value)
            % Converts from ft to m 
            m_value = ft_value/3.2808;
        end
        
        function ms_value = kt_to_ms(kt_value)
            % Converts from knots to m/s
            ms_value = kt_value*0.5144;
        end
        
        function deg_value = rad_to_deg(rad_value)
            % Converts from radians to degrees
            deg_value = rad_value * 180/pi;
        end
        
        function rad_value = deg_to_rad(deg_value)
            % Converts from radians to degrees
            rad_value = deg_value * pi/180;
        end
        
        function kg_value = lb_to_kg(lb_value)
           % Converts from lbs to kgs
           kg_value = lb_value*0.453592;
        end
        
        function kgm2_value = slgft2_tokgm2(slgft2_value)
            % Converts from slugs*ft^2 to kg*m^2
            kgm2_value = slgft2_value*1.35581795;
        end

        function pa_value = lbft2_to_pa(lbft2_value)
            % Converts from lb/ft^2 to Pa
            pa_value = lbft2_value*47.880172;
        end
        
        %% ---- ATMOSPHERE ----
        
        function [T, P, rho] = ISA(h)
            % ISA model. h: meters.
            R = 287.04;    % [m^2/(K*s^2)]
            P_0 = 101325;  % [Pa]
            T_0 = 288.15;  % [K]
            g_0 = 9.80665; % [m/s^2]
            
            if h < 11000
                T = T_0 - 6.5*h/1000;
                P = P_0*(1 - 0.0065*h/T_0)^5.2561;
            else
                T = T_0 - 6.5*11000/1000;
                P_11 = P_0*(1 - 0.0065*11000/T_0)^5.2561;
                P = P_11*exp(-g_0/(R*T)*(h-11000));
            end
            rho = P/(R*T);           
        end
        
        function [h] = lock2h(lock,heRef,atm) % Hechar un ojo a esto no va bien

            rho = ((lock*heRef.mainRotor.IBeta)/((heRef.mainRotor.R^4)*heRef.mainRotor.cldata(1)*heRef.mainRotor.c0));
            fprintf("Rho = " + string(rho));
            h = atm.density2h(rho);

        end
        
        function [lock] = lock(ndHeRef)
            
            %lock = (rho*(heRef.mainRotor.R^4)*heRef.mainRotor.cldata(1)*heRef.mainRotor.c0)/(heRef.mainRotor.IBeta); 

            lock = 8*(ndHeRef.mainRotor.lambdaBeta2 - 1)/ndHeRef.mainRotor.SBeta;

        end

        %% ---- DISPLAYS ---

        function displayTrimState(ndTrimState,PM,name) %Esta hecho un poco a lo burro pero meh
            
            if ~exist('name','var')
                % If lag time is not specified, default to 0.01 s
                name = "------------------";
            end

            Phi = ndTrimState.solution.Phi;
            Theta = ndTrimState.solution.Theta;
            lamda_r = ndTrimState.solution.lambda0;
            lamda_tr = ndTrimState.solution.lambda0tr;
            %*(heRef.tailRotor.Omega*heRef.tailRotor.R)
            theta0 = ndTrimState.solution.theta0;
            theta1C = ndTrimState.solution.theta1C;
            theta1S = ndTrimState.solution.theta1S;
            theta1T = ndTrimState.solution.theta0tr;
            beta0 = ndTrimState.solution.beta0;
            beta1C = ndTrimState.solution.beta1C;
            beta1S = ndTrimState.solution.beta1S;
            CPM = ndTrimState.ndPow.CPM;
            CPra = ndTrimState.ndPow.CPtr;
            CPrp = ndTrimState.ndPow.CPmr;

            
            mg_x = ndTrimState.actions.weight.fuselage.CFx; Frp_x = ndTrimState.actions.mainRotor.fuselage.CFx;
            mg_y = ndTrimState.actions.weight.fuselage.CFy; Frp_y = ndTrimState.actions.mainRotor.fuselage.CFy;
            mg_z = ndTrimState.actions.weight.fuselage.CFz; Frp_z = ndTrimState.actions.mainRotor.fuselage.CFz;

            Fra_x = ndTrimState.actions.tailRotor.fuselage.CFx; Ff_x = ndTrimState.actions.fuselage.fuselage.CFx;
            Fra_y = ndTrimState.actions.tailRotor.fuselage.CFy; Ff_y = ndTrimState.actions.fuselage.fuselage.CFy;
            Fra_z = ndTrimState.actions.tailRotor.fuselage.CFz; Ff_z = ndTrimState.actions.fuselage.fuselage.CFz;

            Fev_x = ndTrimState.actions.verticalFin.fuselage.CFx; 
            Fev_y = ndTrimState.actions.verticalFin.fuselage.CFy;
            Fev_z = ndTrimState.actions.verticalFin.fuselage.CFz;
            
            Feh_x = ndTrimState.actions.rightHTP.fuselage.CFx + ndTrimState.actions.leftHTP.fuselage.CFx;
            Feh_y = ndTrimState.actions.rightHTP.fuselage.CFx + ndTrimState.actions.leftHTP.fuselage.CFy;
            Feh_z = ndTrimState.actions.rightHTP.fuselage.CFx + ndTrimState.actions.leftHTP.fuselage.CFz;

            Ftot_x = abs(mg_x) + abs(Frp_x) + abs(Fra_x) + abs(Ff_x) + abs(Fev_x) + abs(Feh_x);
            Ftot_y = abs(mg_y) + abs(Frp_y) + abs(Fra_y) + abs(Ff_y) + abs(Fev_y) + abs(Feh_y);
            Ftot_z = abs(mg_z) + abs(Frp_z) + abs(Fra_z) + abs(Ff_z) + abs(Fev_z) + abs(Feh_z);

            Mrp_x = ndTrimState.actions.mainRotor.fuselage.CMx; MFrp_x = ndTrimState.actions.mainRotor.fuselage.CMFx;
            Mrp_y = ndTrimState.actions.mainRotor.fuselage.CMy; MFrp_y = ndTrimState.actions.mainRotor.fuselage.CMFy;
            Mrp_z = ndTrimState.actions.mainRotor.fuselage.CMz; MFrp_z = ndTrimState.actions.mainRotor.fuselage.CMFz;
            
            Mra_x = ndTrimState.actions.tailRotor.fuselage.CMx; MFra_x = ndTrimState.actions.tailRotor.fuselage.CMFx;
            Mra_y = ndTrimState.actions.tailRotor.fuselage.CMy; MFra_y = ndTrimState.actions.tailRotor.fuselage.CMFy;
            Mra_z = ndTrimState.actions.tailRotor.fuselage.CMz; MFra_z = ndTrimState.actions.tailRotor.fuselage.CMFz;

            Mf_x = ndTrimState.actions.fuselage.fuselage.CMx;  MFf_x = ndTrimState.actions.fuselage.fuselage.CMFx;
            Mf_y = ndTrimState.actions.fuselage.fuselage.CMy;  MFf_y = ndTrimState.actions.fuselage.fuselage.CMFy;
            Mf_z = ndTrimState.actions.fuselage.fuselage.CMz;  MFf_z = ndTrimState.actions.fuselage.fuselage.CMFz;
            
            Mev_x = ndTrimState.actions.verticalFin.fuselage.CMx; MFev_x = ndTrimState.actions.verticalFin.fuselage.CMFx;
            Mev_y = ndTrimState.actions.verticalFin.fuselage.CMy; MFev_y = ndTrimState.actions.verticalFin.fuselage.CMFy;
            Mev_z = ndTrimState.actions.verticalFin.fuselage.CMz; MFev_z = ndTrimState.actions.verticalFin.fuselage.CMFz;
            
            Meh_x = ndTrimState.actions.rightHTP.fuselage.CMx + ndTrimState.actions.leftHTP.fuselage.CMx;
            Meh_y = ndTrimState.actions.rightHTP.fuselage.CMy + ndTrimState.actions.leftHTP.fuselage.CMy;
            Meh_z = ndTrimState.actions.rightHTP.fuselage.CMz + ndTrimState.actions.leftHTP.fuselage.CMz;

            MFeh_x = ndTrimState.actions.rightHTP.fuselage.CMFx + ndTrimState.actions.leftHTP.fuselage.CMFx;
            MFeh_y = ndTrimState.actions.rightHTP.fuselage.CMFy + ndTrimState.actions.leftHTP.fuselage.CMFy;
            MFeh_z = ndTrimState.actions.rightHTP.fuselage.CMFz + ndTrimState.actions.leftHTP.fuselage.CMFz;

            Mtot_x = abs(Mrp_x) + abs(Mra_x) + abs(Mf_x) + abs(Mev_x) + abs(Meh_x);
            Mtot_y = abs(Mrp_y) + abs(Mra_y) + abs(Mf_y) + abs(Mev_y) + abs(Meh_y);
            Mtot_z = abs(Mrp_z) + abs(Mra_z) + abs(Mf_z) + abs(Mev_z) + abs(Meh_z);

            MFtot_x = abs(MFrp_x) + abs(MFra_x) + abs(MFf_x) + abs(MFev_x) + abs(MFeh_x);
            MFtot_y = abs(MFrp_y) + abs(MFra_y) + abs(MFf_y) + abs(MFev_y) + abs(MFeh_y);
            MFtot_z = abs(MFrp_z) + abs(MFra_z) + abs(MFf_z) + abs(MFev_z) + abs(MFeh_z);

            
            disp("------------- " + name + " -------------");
            disp("------------------- TRIM SOLUTIONS --------------------");
            disp("Phi: " + string(Phi*(180/pi)) + " º");
            disp("Theta: " + string(Theta*(180/pi)) + " º");
            disp("Lambda_r (V_inducida Rotor_p): " + string(lamda_r));
            disp("Lambda_tr (V_inducida Rotor_tail): " + string(lamda_tr));
            disp("Theta 0: " + string(theta0*(180/pi)) + " º");
            disp("Theta 1C: " + string(theta1C*(180/pi)) + " º");
            disp("Theta 1S: " + string(theta1S*(180/pi)) + " º");
            disp("Theta 1T: " + string(theta1T*(180/pi)) + " º");
            disp("Beta 0: " + string(beta0*(180/pi)) + " º");
            disp("Beta 1C: " + string(beta1C*(180/pi)) + " º");
            disp("Beta 1S: " + string(beta1S*(180/pi)) + " º");
            disp("---------------------- POWER COEFS -----------------------");
            disp("Coeff P_M: " + string(CPM));
            disp("P_M: " + string(PM) + " W");
            disp("Coeff P_mr (CP_rp): " + string(CPrp));
            disp("Coeff P_tr (CP_ra): " + string(CPra));
            disp("P_ra/P_rp: " + string(CPra/CPrp));
            disp("ACTIVE CONTRIBUTIONS IN BODY AXES -------------------------");
            disp("------- FORCES COEFFS -------")
            disp("ELEMENT    | x [%] | y [%] | z [%] "); format default
            disp("Weight     | " + string(round(100*(mg_x/Ftot_x),1)) + " | " + string(round(100*mg_y/Ftot_y,1)) + " | " + string(round(100*mg_z/Ftot_z,1)));
            disp("MainRotor  | " + string(round(100*Frp_x/Ftot_x,1)) + " | " + string(round(100*Frp_y/Ftot_y,1)) + " | " + string(round(100*Frp_z/Ftot_z,1)));
            disp("TailRotor  | " + string(round(100*Fra_x/Ftot_x,1)) + " | " + string(round(100*Fra_y/Ftot_y,1)) + " | " + string(round(100*Fra_z/Ftot_z,1)));
            disp("Fuselage   | " + string(round(100*Ff_x/Ftot_x,1)) + " | " + string(round(100*Ff_y/Ftot_y,1)) + " | " + string(round(100*Ff_z/Ftot_z,1)));
            disp("Vertical E | " + string(round(100*Fev_x/Ftot_x,1)) + " | " + string(round(100*Fev_y/Ftot_y,1)) + " | " + string(round(100*Fev_z/Ftot_z,1)));
            disp("HorizontalE| " + string(round(100*Feh_x/Ftot_x,1)) + " | " + string(round(100*Feh_y/Ftot_y,1)) + " | " + string(round(100*Feh_z/Ftot_z,1)));
            disp("--------------------------------------------------------------------------------");
            disp("TOTAL FORCE| " + string(round(Ftot_x,3,'significant')) + " | " + string(round(Ftot_y,3,'significant')) + " | " + string(round(Ftot_y,3,'significant')));
            disp("--------------------------------------------------------------------------------");
            disp("------- MOMENT COEFFS -------");
            disp("ELEMENT    | TYPE   | x [%] | y [%] | z [%] ");
            disp("MainRotor  | Mrp    | " + string(round(100*Mrp_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*Mrp_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*Mrp_z/(Mtot_z + MFtot_z),1)));
            disp("           | OAxFrp | " + string(round(100*MFrp_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*MFrp_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*MFrp_z/(Mtot_z + MFtot_z),1)));
            disp("TailRotor  | Mra    | " + string(round(100*Mra_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*Mra_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*Mra_z/(Mtot_z + MFtot_z),1)));
            disp("           | OAxFra | " + string(round(100*MFra_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*MFra_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*MFra_z/(Mtot_z + MFtot_z),1)));
            disp("Fuselage   | Mf     | " + string(round(100*Mf_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*Mf_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*Mf_z/(Mtot_z + MFtot_z),1)));
            disp("           | OAxFf  | " + string(round(100*MFf_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*MFf_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*MFf_z/(Mtot_z + MFtot_z),1)));
            disp("Vertical E | Mev    | " + string(round(100*Mev_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*Mev_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*Mev_z/(Mtot_z + MFtot_z),1)));
            disp("           | OAxFev | " + string(round(100*MFev_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*MFev_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*MFev_z/(Mtot_z + MFtot_z),1)));
            disp("HorizontalE| Meh    | " + string(round(100*Meh_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*Meh_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*Meh_z/(Mtot_z + MFtot_z),1)));
            disp("           | OAxFeh | " + string(round(100*MFeh_x/(Mtot_x + MFtot_x),1)) + " | " + string(round(100*MFeh_y/(Mtot_y + MFtot_y),1)) + " | " + string(round(100*MFeh_z/(Mtot_z + MFtot_z),1)));
            disp("--------------------------------------------------------------------------------");
            disp("TOTAL M     | " + string(round(Mtot_x,3,'significant')) + " | " + string(round(Mtot_y,3,'significant')) + " | " + string(round(Mtot_y,3,'significant')));
            disp("TOTAL OAxF  | " + string(round(MFtot_x,3,'significant')) + " | " + string(round(MFtot_y,3,'significant')) + " | " + string(round(MFtot_y,3,'significant')));
            disp("TOTAL M+OAxF| " + string(round(Mtot_x + MFtot_x ,3,'significant')) + " | " + string(round(Mtot_y + MFtot_y,3,'significant')) + " | " + string(round(Mtot_y + MFtot_x,3,'significant')));
            disp("--------------------------------------------------------------------------------");
       
        end

        function [k,cdo] = getkcdoDependency(heRef,c1,c0,sigma)

            rho0 = 1.225; %kg/m^3
            
            k = c1*sqrt(2*rho0*pi*heRef.mainRotor.R^2);
            cdo = (8*c0)/(rho0*(heRef.mainRotor.R^5)*(heRef.mainRotor.Omega^3)*pi*sigma);

        end

        function [heRef, ndHeRef, atm, options] = createBo105(hsl,autoRot,initialconditions,source)

            if ~exist('hsl','var')

                hsl = 0;
            end

            if ~exist('autoRot','var')

                autoRot = false;
            end

            if ~exist('initialconditions','var')
            
                ic = false;
            else
                ic = true;
            end


            % Creamos el he Bo105 a punto fijo y nivel del mar
            
            atm               = getISA;
            heRef             = PadfieldBo105(atm);
            ndHeRef           = rigidHe2ndHe(heRef,atm,hsl);
            opt               = optimset('Display','off');
            options           = setHeroesRigidOptions;
            options.armonicInflowModel = @none;
            
            % default optiosns are set
            options = setHeroesRigidOptions;
            
            %engineState
            %options.engineState = @mainShaftBroken;
            
            % uniformInflowModel options are updated with
            % @Cuerva model for induced velocity
            %options.uniformInflowModel = @Cuerva;
            %options.armonicInflowModel = @none;
            
            % engineState options are updated for the 
            if autoRot == true

                %options.engineState = @EngineOffTransmissionOn;
                options.engineState = @mainShaftBroken;

            end
            
            if ic == true
                
                options.IniTrimCon  = initialconditions;
            end

            % mrForces options are updated for 
            %options.mrForces = @completeF;
            options.mrForces =@thrustF;
            
            % trForces options are updated
            options.trForces = @completeF;

            if ~exist('source','var')
            
                % uniformInflowModel options are updated with
                % @Cuerva model for induced velocity
                options.uniformInflowModel = @Cuerva;
                options.armonicInflowModel = @none;
                
            elseif exist('source','var')
                
                if source == "Rand"
    
                    % @Rand model for induced velocity
                    options.uniformInflowModel = @Rand;
                    options.armonicInflowModel = @none;
    
                elseif source == "Glauert"
                    
                    % @Glauert model for induced velocity
                    options.uniformInflowModel = @Glauert;
                    options.armonicInflowModel = @none;
    

                elseif source == "Cuerva"
                    
                    % @Cuerva model for induced velocity
                    options.uniformInflowModel = @Cuerva;
                    options.armonicInflowModel = @none;
    
                end
            end

        end

        function [P_max_alcance, P_max_autonm,V_max_alcance, V_max_autonm] = getHePerformance(dimTrimState,VHs,display)
            
            % Máximo alcance: tangente a la curva que pasa por el origen: min(P(V)/V)
            % Máxima autonomía: mínimo de la curva de potencia 

            if ~exist('display','var')

                display = false;
            end
            
            P_V = zeros(length(dimTrimState.Pow.PM(:,1)),length(dimTrimState.Pow.PM(1,:)));

            for j = 1:length(P_V(:,1))
                for h = 1:length(P_V(1,:))

                    P_V(j,h) = dimTrimState.Pow.PM(j,h)/VHs(j);
                end
            end

            P_max_autonm = zeros(length(dimTrimState.Pow.PM(1,:)),1); V_max_autonm = zeros(length(dimTrimState.Pow.PM(1,:)),1);
            P_max_alcance = zeros(length(dimTrimState.Pow.PM(1,:)),1); V_max_alcance = zeros(length(dimTrimState.Pow.PM(1,:)),1);
            
            for i = 1:length(dimTrimState.Pow.PM(1,:))
                
                [PmaxAut,iPmaxAut] = min(dimTrimState.Pow.PM(:,i));
                [~,iPmaxAlc] = min(P_V(:,i));

                P_max_autonm(i) = PmaxAut; P_max_alcance(i) =  dimTrimState.Pow.PM(iPmaxAlc,i);
                V_max_autonm(i) = VHs(iPmaxAut); V_max_alcance(i) = VHs(iPmaxAlc);
            end

            if display == true
                disp("----------------- HE PERFORMANCES -------------------");
                 for i = 1:length(dimTrimState.Pow.PM(1,:))
                    
                    disp("------ Variable value i = " + string(i) + " -----------");
                    disp("P_maxRange = " + string(P_max_alcance(i)) + " W");
                    disp("----> V_maxRange = " + string(V_max_alcance(i)) + " m/s");
                    disp("P_maxAutonomy = " + string(P_max_autonm(i)) + " W");
                    disp("----> V_maxAutonomy  = " + string(V_max_autonm(i)) + " m/s");
                 end
            end

        end

    end
end

