classdef record_beluga < handle
    
    properties
        output_history
        output_time_history
        input_history
        input_time_history
        ws
        
        start_time = NaN;
        
        robot
        
        c
        
        out_lh
        in_lh
    end
    
    methods
        
        function obj = record_beluga(robot_name)
            
            obj.robot = robot_name;
            
            obj.c = clock();
           
            obj.ws = ros_websocket('ws://localhost:9090');
            output_sub = Subscriber(obj.ws, '/state_estimate', 'dcsl_messages/StateArray');
            input_sub = Subscriber(obj.ws, '/robot0/cmd_inputs', 'dcsl_messages/belugaInput');
            obj.out_lh = event.listener(output_sub, 'OnMessageReceived', @(h,e) obj.output_callback(h, e));
            obj.in_lh = event.listener(input_sub, 'OnMessageReceived', @(h,e) obj.input_callback(h, e));
            
        end
        
        function output_callback(obj, ~, e)
            obj.output_history(end+1,:) = [e.data.states{1}.pose.position.x e.data.states{1}.pose.position.y e.data.states{1}.pose.position.z sin(e.data.states{1}.pose.orientation.z) cos(e.data.states{1}.pose.orientation.z)];
            ros_time = e.data.header.stamp.secs + 10^-9 * e.data.header.stamp.nsecs;
            
            if (isnan(obj.start_time))
                obj.start_time = ros_time;
            end
                
            obj.output_time_history(end+1) = ros_time - obj.start_time;
        end
        
        function input_callback(obj, ~, e)
            obj.input_history(end+1,:) = [e.data.thrust_motor, e.data.servo e.data.vertical_motor];
            ros_time = e.data.header.stamp.secs + 10^-9 * e.data.header.stamp.nsecs;
            
            if (isnan(obj.start_time))
                obj.start_time = ros_time;
            end
            
            obj.input_time_history(end+1) = ros_time - obj.start_time;
        end
            
        function close(obj)
            obj.ws.close();
            
            pause(0.1)
            
            threshold = 0.008;
            
            % Make both input and output histories start at the same ROS
            % time mark
            if obj.input_time_history(1) < obj.output_time_history(1)
                while true
                    if abs(obj.input_time_history(1) - obj.output_time_history(1)) < threshold
                        break
                    else
                        obj.input_time_history = obj.input_time_history(2:end);
                        obj.input_history = obj.input_history(2:end,:);
                    end
                end
            else
                while true
                    if abs(obj.output_time_history(1) - obj.input_time_history(1)) < threshold
                        break
                    else
                        obj.output_time_history = obj.output_time_history(2:end);
                        obj.output_history = obj.output_history(2:end, :);
                    end
                end
            end
            % Make sure both histories are the same number of time steps.
            if size(obj.input_time_history, 2) < size(obj.output_time_history,2)
                obj.output_time_history = obj.output_time_history(1:size(obj.input_time_history,2));
                obj.output_history = obj.output_history(1:size(obj.input_time_history,2),:);
            elseif size(obj.output_time_history,2) < size(obj.input_time_history,2)
                obj.input_time_history = obj.input_time_history(1:size(obj.output_time_history,2));
                obj.input_history = obj.input_history(1:size(obj.output_time_history,2),:);
            end
            
            % Create iddata object
            ts_data = iddata(obj.output_history, obj.input_history, [], 'SamplingInstants', obj.output_time_history,...
                'OutputName', {'x-position', 'y-position', 'z-position', 'sin(\theta)', 'cos(\theta)'},...
                'OutputUnit', {'m', 'm', 'm', 'units', 'units'},...
                'InputName', {'u_t', 'u_{\phi}', 'u_z'},...
                'InputUnit', {'counts', 'radians', 'counts'});
            
            % Create time stamped file name and save iddata object to file
            c = obj.c;            
            fname = strcat('data/', obj.robot, num2str(c(1)), '_', num2str(c(2)), '_', num2str(c(3)), '_', num2str(c(4)), '_', num2str(c(5)), '_', num2str(round(c(6))), '.mat');
            save(fname, 'ts_data')
            
        end
        
    end

end