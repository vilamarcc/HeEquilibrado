function io = test_myplotNdTrimState(mode)

close all

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


% atmosphere variables needed
atm     = getISA;

% helicopter model selection
he      = rigidBo105(atm);

% helicopter 2 non-dimensional helicopter
H       = 0;
ndHe    = rigidHe2ndHe(he,atm,H);

% wind velocity in ground reference system
muWT = [0; 0; 0];

tic

FC = {'VOR',linspace(0.05,0.3,5),...
      'betaf0',0,...
      'gammaT',pi/180*[0,5,10],...
      'cs',0,...
      'vTOR',0};

ndTrimState = getNdHeTrimState(ndHe,muWT,FC,options);

toc

axds           = getaxds({'VOR'},{'$V/(\Omega R)$ [-]'},1);
ayds           = getaxds('gammaT','$\gamma_{T}[\mathrm{^o}]$',180/pi);


% Use of plotNdTrimSolution using default variables (plots everything)
axATa          = plotNdTrimSolution(ndTrimState.solution,...
                 axds,ayds,'defaultVars','yes',...
                 'plot3dMode','parametric');               


% % Next, plot using the bidimensional data using contour             
% axATa          = plotNdTrimSolution(ndTrimState.solution,axds,ayds,'defaultVars','yes',...
%                  'plot3dMethod',@contour);               

% Next, plot selecting nondimensional variables
azds           = getaxds(...
                 {'Phi','Theta',...
                  'theta0','theta1C',...
                  'theta1S','theta0tr'...
                  },...
                  {'$\Phi [\mathrm{^o}]$,','$\Theta [\mathrm{^o}]$,',...
                   '$\theta_0 [\mathrm{^o}]$','$\theta_{1C} [\mathrm{^o}]$',...
                   '$\theta_{1S} [\mathrm{^o}$]','$\theta_{T} [\mathrm{^o}$]'...
                  },...
                  [180/pi,180/pi,...
                   180/pi,180/pi,...
                   180/pi,180/pi,...
                   180/pi,180/pi,...
                   180/pi ...
                   ]);
               
axATa          = plotNdTrimSolution(ndTrimState.solution,...
                 axds,ayds,'defaultVars',azds,...
                 'plot3dMode','parametric');               

% Next plot dimensional state

trimState      = ndHeTrimState2HeTrimState(ndTrimState,he,atm,H,options);
dimPower       = struct(...
                 'V', trimState.solution.V,...
                 'PM',trimState.Pow.PM,...
                 'gammaT',ndTrimState.solution.gammaT ...
                 );
axds           = getaxds({'V'},{'$V$ [m/s]'},1);
ayds           = getaxds('gammaT','$\gamma_{T}[\mathrm{^o}]$',180/pi);
azds           = getaxds({'PM'},{'$P_M$[kW]'},1e3);
axATb          = plotNdTrimSolution(dimPower,axds,ayds,...
                 'defaultVars',azds,...
                 'plot3dMode','parametric');               


io             = 1;

end