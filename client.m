function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',60); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;

%Dictionary mapping to help with displaying modes
MODES = containers.Map({0,1,2,3,4}, {'IDLE','PWM','ITEST','HOLD','TRACK'});

% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('     y: Dummy Command   x: Add Command  q: Quit\r\n    c: Get Encoder Count   d: Get Encoder Angle  e: Reset Encoder Value\r\n      r: Get Mode of PIC32       a: Read current sensor (ADC counts)         b: Read current sensor (mA)\r\n      f: Set PWM       p: Unpower the Motor     g: Set current gains    h: Get current gains\r\n       k: Test current gains      i: Set position gains    j: Get position gains\r\n   l: Set desired angle (deg)  m: Get step trajectory    n: Get cubic trajectory     o:Execute trajectory \n ');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'y'                         % example operation
            n = input('Enter number: '); % get the number to send
            fprintf(mySerial, '%d\n',n); % send the number
            n = fscanf(mySerial,'%d');   % get the incremented number back
            fprintf('Read: %d\n',n);     % print it to the screen
        case 'x'                         % example operation
            n = input('Enter 2 numbers in array to add: '); % get the numbers to send
            fprintf(mySerial, '%d %d\n',n); % send the numbers
            n = fscanf(mySerial,'%d', n);   % get the sum of the numbers back
            fprintf('Read: %d\n',n);     % print it to the screen
        case 'q'
            has_quit = true;             % exit client
        case 'c'
            counts = fscanf(mySerial,'%d');   % get the encoder counts back
            fprintf('The motor angle is %d counts.\n',counts);     % print it to the screen
        case 'd'
            angle = fscanf(mySerial,'%f');   % get the encoder counts in degrees back
            fprintf('The motor angle is %d degrees.\n',angle);     % print it to the screen
        case 'e'
            res_val = fscanf(mySerial,'%d');   % get the encoder counts back
            fprintf('The motor angle has been reset to %d counts.\n',res_val);     % print it to the screen
        case 'r'
            mode = fscanf(mySerial, '%d');  %read the numeric value of the mode
            fprintf('PIC32 currently set to: %s\n', MODES(mode));     %display the current mode
        case 'a'
            adc_counts = fscanf(mySerial,'%d');   % get the adc counts back
            fprintf('The current sensor reads %d counts.\n',adc_counts);     % print it to the screen
        case 'b'
            current = fscanf(mySerial,'%d');   % get the adc counts in mA back
            fprintf('The current sensor reads %d mA.\n',current);     % print it to the screen
        case 'f'
            pwm = input('Set PWM (-100 to 100): ');      %Get input
            fprintf(mySerial,'%d\n', pwm);                      %Send to the PIC
        case 'p'
            fprintf('The motor is being unpowered.\n');
        case 'g'                         % Set current gains
            gains = input('Enter Kp and Ki for current control in this format[Kp Ki]: '); % get the numbers to send
            fprintf(mySerial, '%f %f\n',gains); % send the numbers
        case 'h'
            gains = fscanf(mySerial,'%f');   % get the current gains back
            fprintf('Kp: %f Ki: %f\n',gains(1), gains(2));     % print it to the screen
        case 'k' % Test current gains
            data = read_plot_matrix(mySerial);
        case 'i'                         % Set position gains
            gains = input('Enter Kp, Ki and Kd for current control in this format[Kp Ki Kd]: '); % get the numbers to send
            fprintf(mySerial, '%f %f %f\n',gains); % send the numbers
        case 'j'
            gains = fscanf(mySerial,'%f');   % get the position gains back
            fprintf('Kp: %f Ki: %f Kd: %f\n',gains(1), gains(2), gains(3));     % print it to the screen
        case 'l'                         % Set Desired Angle
            desAng = input('Enter your desired angle in degrees: '); % get the numbers to send
            fprintf(mySerial, '%d\n',desAng); % send the numbers 
        case 'm'                         % Generate step trajectory
            A = input('Enter step trajectory: ');
            ref = genRef(A, 'step'); % get the numbers to send
            fprintf(mySerial, '%d\n', length(ref));  %sends the number of samples N to the PIC32,
            for i=1:length(ref)                    %sends N reference positions
                fprintf(mySerial, '%f\n', ref(i));
            end
%             n = fscanf(mySerial,'%d');   % get the incremented number back
%             fprintf('Read: %d\n',n);     % print it to the screen
        case 'n'                         % Set Desired Angle
            A = input('Enter cubic trajectory: ');
            ref = genRef(A, 'cubic'); % get the numbers to send
            length(ref)
            fprintf(mySerial, '%d\n', length(ref));  %sends the number of samples N to the PIC32,
            for i=1:length(ref)                    %sends N reference positions
                fprintf(mySerial, '%f\n', ref(i));
            end
        case 'o'
            nsamples = fscanf(mySerial, '%d');
            data = zeros(nsamples,2);
            for i=1:nsamples
                data(i,:) = fscanf(mySerial, '%f %f');
                t(i) = (i-1)*0.005;
            end
            if nsamples > 1
                plot(t, data(:,1:2));
            else
                fprintf('Only 1 sample received\n');
            end
            
            title(sprintf('Performance'));
            ylabel('Position');
            xlabel('Time (ms)');
            fprintf('\n');
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
