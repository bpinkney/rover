function status = parse_flightlog_06(varargin)

global block_len;
block_len = 10000;

%% --------------------------------------------------------
%% Open the text files 
%% --------------------------------------------------------

for f=1:length(varargin)
  filename = char(varargin(f));
  status = 0;
  [fid, err_msg]= fopen(filename);
  if(fid < 0)
    display(sprintf('Failed opening %s: %s', filename, err_msg));
    status = -1;
    return;
  end

%% --------------------------------------------------------
%% Read and parse each line 
%% --------------------------------------------------------
  while (1)
    theline = fgetl(fid);
    if isempty(theline)
        continue;
    end
    if(theline == -1)
        break;
    end
    
    % Skip comments
    if(~isempty(findstr(theline, '#')))
        continue;
    end
    % Remove spaces
    %length(theline)
%     for i=2:length(theline)-1
%         if(theline(i) == ' ')
%             theline(i) = '0';
%         end
%     end

    if(~isempty(findstr(theline, 'DATA')))
      parse_data_line(theline);
    end
    
  end

  if( fid >= 0 )
    fclose(fid);
  end
  
end

function parse_data_line(theline)
  global flight_log_data;
  global block_len;

  % Initialize the struct 
  if isempty(flight_log_data)
    flight_log_data.idx = 0;
    flight_log_data.rover_t = zeros(1,1);  
	flight_log_data.rover_status = zeros(1,1); 
	flight_log_data.rover_orient = zeros(1,3);  
    flight_log_data.rover_orient_des = zeros(1,3); 
    flight_log_data.rover_rates_des = zeros(1,3); 
    flight_log_data.rover_accs_est = zeros(1,3);
	flight_log_data.rover_ex_gyro = zeros(1,3);  
    flight_log_data.rover_ex_gyro_temp = zeros(1,1);
	flight_log_data.rover_ex_acc = zeros(1,3);  
	flight_log_data.rover_int_acc = zeros(1,3);  
	flight_log_data.rover_ex_mag = zeros(1,3);  
	flight_log_data.rover_int_mag = zeros(1,3); 
    flight_log_data.rover_m_thrust = zeros(1,4); 

  end

% Expand the arrays if we're out of space 
  flight_log_data.idx = flight_log_data.idx + 1;
%   if flight_log_data.idx > length(flight_log_data.rover_t)
%     flight_log_data.rover_t = [flight_log_data.rover_t; zeros(block_len, 1)];   
% 	flight_log_data.rover_status = [flight_log_data.rover_status; zeros(block_len, 1)];	
% 	flight_log_data.rover_orient = [flight_log_data.rover_orient; zeros(block_len, 3)];	
% 	flight_log_data.rover_ex_gyro = [flight_log_data.rover_ex_gyro; zeros(block_len, 3)];	
%     flight_log_data.rover_ex_gyro_temp = [flight_log_data.rover_ex_gyro_temp; zeros(block_len, 1)];
% 	flight_log_data.rover_ex_acc = [flight_log_data.rover_ex_acc; zeros(block_len, 3)];	
% 	flight_log_data.rover_int_acc = [flight_log_data.rover_int_acc; zeros(block_len, 3)];	
% 	flight_log_data.rover_ex_mag = [flight_log_data.rover_ex_mag; zeros(block_len, 3)];	
% 	flight_log_data.rover_int_mag = [flight_log_data.rover_int_mag; zeros(block_len, 3)];	
%   end

% Parse the string into the struct
  [flight_log_data.rover_t(flight_log_data.idx, 1), ...
   flight_log_data.rover_status(flight_log_data.idx, 1), ...
   flight_log_data.rover_orient(flight_log_data.idx, 1), ...
   flight_log_data.rover_orient(flight_log_data.idx, 2), ...
   flight_log_data.rover_orient(flight_log_data.idx, 3), ...
   flight_log_data.rover_orient_des(flight_log_data.idx, 1), ...
   flight_log_data.rover_orient_des(flight_log_data.idx, 2), ...
   flight_log_data.rover_orient_des(flight_log_data.idx, 3), ...
   flight_log_data.rover_rates_des(flight_log_data.idx, 1), ...
   flight_log_data.rover_rates_des(flight_log_data.idx, 2), ...
   flight_log_data.rover_rates_des(flight_log_data.idx, 3), ...
   flight_log_data.rover_accs_est(flight_log_data.idx, 1), ...
   flight_log_data.rover_accs_est(flight_log_data.idx, 2), ...
   flight_log_data.rover_accs_est(flight_log_data.idx, 3), ...
   flight_log_data.rover_ex_gyro(flight_log_data.idx, 1), ...
   flight_log_data.rover_ex_gyro(flight_log_data.idx, 2), ...
   flight_log_data.rover_ex_gyro(flight_log_data.idx, 3), ...
   flight_log_data.rover_ex_gyro_temp(flight_log_data.idx, 1), ...
   flight_log_data.rover_ex_acc(flight_log_data.idx, 1), ...
   flight_log_data.rover_ex_acc(flight_log_data.idx, 2), ...
   flight_log_data.rover_ex_acc(flight_log_data.idx, 3), ...
   flight_log_data.rover_int_acc(flight_log_data.idx, 1), ...
   flight_log_data.rover_int_acc(flight_log_data.idx, 2), ...
   flight_log_data.rover_int_acc(flight_log_data.idx, 3), ...
   flight_log_data.rover_ex_mag(flight_log_data.idx, 1), ...
   flight_log_data.rover_ex_mag(flight_log_data.idx, 2), ...
   flight_log_data.rover_ex_mag(flight_log_data.idx, 3), ...
   flight_log_data.rover_int_mag(flight_log_data.idx, 1), ...
   flight_log_data.rover_int_mag(flight_log_data.idx, 2), ...
   flight_log_data.rover_int_mag(flight_log_data.idx, 3), ...
   flight_log_data.rover_m_thrust(flight_log_data.idx, 1), ...
   flight_log_data.rover_m_thrust(flight_log_data.idx, 2), ...
   flight_log_data.rover_m_thrust(flight_log_data.idx, 3), ...
   flight_log_data.rover_m_thrust(flight_log_data.idx, 4)] = ...
    strread(theline, 'DATA[%u %u [%n%n%n] [%n%n%n] [%n%n%n] [%n%n%n] [%n %n %n] %d [%n %n %n] [%n %n %n] [%n %n %n] [%n %n %n] [%n %n %n %n]]', 'delimiter', ',');
% end
%'DATA[%n, %n, [%n,%n,%n], [%n, %n, %n], [%n, %n, %n], [%n, %n, %n], [%n, %n, %n], [%n, %n, %n]]', 'delimiter', ',');
% end

% ----------------------------------------------------
% Clean up structure for flight_log_data from gimbal_control_msgs.h:16
function cleanup_flight_log_data(clear_all)
  global flight_log_data;
  if isempty(flight_log_data) | clear_all 
    clear global flight_log_data;
    return;
  end
  clear_range = max(flight_log_data.idx+1, 1):length(flight_log_data.rover_t);
  flight_log_data.rover_t(clear_range, :) = [];  
%end

