%% Model based on
% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_sensorless_f28069MLaunchPad_datascript.m

%% Gather the dataset

%data = out.simout.signals.values;   put in command window, then save array 

%% Set PWM Switching frequency
PWM_frequency 	= 30e3;             %Hz  // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s   // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // Sample time step for controller
Ts_simulink     = T_pwm/2;      %sec        // Simulation time step for model simulation
Ts_motor        = T_pwm/2;      %Sec        // Simulation sample time
Ts_inverter     = T_pwm/2;      %sec        // Simulation time step for average value inverter
Ts_speed        = 30*Ts;        %Sec        // Sample time for speed controller

%% Set data type for controller & code-gen
dataType = 'single';            % Floating point code-generation

%% System Parameters // Hardware parameters

% Set Target Parameters
target.CPU_frequency        = 84e6;					%Hz     // Clock frequency
target.PWM_frequency        = PWM_frequency;   		%Hz     // PWM frequency
target.PWM_Counter_Period   = round(target.CPU_frequency/target.PWM_frequency/2); % //PWM timer counts for up-down counter
target.ADC_Vref             = 3.3;					%V		// ADC voltage reference for LAUNCHXL-F28379D
target.ADC_MaxCount         = 4095;					%		// Max count for 12 bit ADC

%% Motor Parameters ()
pmsm.p      = 4;                %           // Pole Pairs for the motor
pmsm.Rs     = 0.4;              %Ohm        // Stator Resistor
pmsm.Ld     = 0.6e-3;           %H          // D-axis inductance value
pmsm.Lq     = 0.6e-3;           %H          // Q-axis inductance value
pmsm.J      = 4.8018552467e-06; %Kg-m2      // Inertia in SI units
pmsm.B      = 9.6037104933e-06; %Kg-m2/s    // Friction Co-efficient
pmsm.Ke     = 4;                %Bemf Const	// Vpk_LL/krpm
pmsm.Kt     = 0.04103;          %Nm/A       // Torque constant
pmsm.I_rated= 3.5;              %A      	// Rated current (phase-peak)
pmsm.N_max  = 10000;            %rpm        // Max speed
pmsm.PositionOffset = 0.1712;	%PU position// Position Offset
pmsm.QEPSlits       = 1250;     %           // QEP Encoder Slits
pmsm.FluxPM     = (pmsm.Ke)/(sqrt(3)*2*pi*1000*pmsm.p/60); %PM flux computed from Ke
pmsm.T_rated    = (3/2)*pmsm.p*pmsm.FluxPM*pmsm.I_rated;   %Get T_rated from I_rated

pmsm.N_base = 4000; % Motor base speed
pmsm.PositionOffset = 0.1917; % Sensorless not required

%% Set inverter details (X-NUCLEO-IHM07M)

inverter.V_dc          = 24;       				%V      // DC Link Voltage of the Inverter (Required for motor)
inverter.I_trip        = 3.55;       			%Amps   // Max current for trip
inverter.Rds_on        = 2e-3;     				%Ohms   // Rds ON for X-NUCLEO-IHM07M1 (From motor driver chip)
inverter.Rshunt        = 0.33;    				%Ohms   // Rshunt for X-NUCLEO-IHM07M1
inverter.CtSensAOffset = 1665;        			%Counts // ADC Offset for phase-A
inverter.CtSensBOffset = 1943;        			%Counts // ADC Offset for phase-B
inverter.ADCGain       = 1;                     %       // ADC Gain factor scaled by SPI (On board amplification is not possible)
inverter.EnableLogic   = 1;    					% 		// Active high for X-NUCLEO-IHM07M1 enable pin (EN_GATE)
inverter.invertingAmp  = -1;   					% 		// Non inverting current measurement amplifier
inverter.ISenseVref    = 3.3;					%V 		// Voltage ref of inverter current sense circuit
inverter.ISenseVoltPerAmp = 0.505; 				%V/Amps // Current sense voltage output per 1 A current (Rshunt * iSense op-amp gain)
inverter.ISenseMax     = inverter.ISenseVref/(2*inverter.ISenseVoltPerAmp); %Amps // Maximum Peak-Neutral current that can be measured by inverter current sense
inverter.R_board       = inverter.Rds_on + inverter.Rshunt/3;  %Ohms
inverter.ADCOffsetCalibEnable = false;

%% PU System details // Set base values for pu conversion
PU_System.V_base   = (inverter.V_dc/sqrt(3));
PU_System.I_base   = inverter.ISenseMax;
PU_System.N_base   = pmsm.N_base;
PU_System.T_base   = (3/2)*pmsm.p*pmsm.FluxPM*PU_System.I_base;
PU_System.P_base   = (3/2)*PU_System.V_base*PU_System.I_base;

acceleration = 20000/PU_System.N_base;                  %  P.U/Sec // Maximum allowable acceleraton

%% Open loop reference values
T_Ref_openLoop          = 1;                    % Sec // Time for open-loop start-up
Speed_openLoop_PU       = 0.1;                  % PU  // Per-Unit speed referene for open-loop start-up
Vd_Ref_openLoop_PU      = Speed_openLoop_PU*2;  % Use 1.2x for Dyno setup and 2x for others

%% Controller design 
PI_params = mcb.internal.SetControllerParameters(pmsm,inverter,PU_System,T_pwm,2*Ts,Ts_speed);

%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1      = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

% Convert all parameters of datatype double to single
%% Displaying model variables
disp(pmsm);
disp(inverter);
disp(target);
