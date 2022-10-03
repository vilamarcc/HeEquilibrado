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

    end
end

