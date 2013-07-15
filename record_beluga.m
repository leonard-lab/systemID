classdef record_beluga < handle
    
    properties
        output_history
        output_time_history
        input_history
        input_time_history
        ws
        
        out_lh
        in_lh
    end
    
    methods
        
        function obj = record_beluga()
            
            obj.ws = ros_websocket('ws://localhost:9090');
            output_sub = Subscriber(obj.ws, '/state_estimate', 'dcsl_messages/StateArray');
            input_sub = Subscriber(obj.ws, '/robot0/cmd_inputs', 'dcsl_messages/belugaInput');
            obj.out_lh = event.listener(output_sub, 'OnMessageReceived', @(h,e) obj.output_callback(h, e));
            obj.in_lh = event.listener(input_sub, 'OnMessageReceived', @(h,e) obj.input_callback(h, e));
            
        end
        
        function output_callback(obj, ~, e)
            obj.output_history(end+1,:) = [e.data.states{1}.pose.position.x e.data.states{1}.pose.position.y e.data.states{1}.pose.position.z e.data.states{1}.pose.orientation.z];
            ros_time = e.data.header.stamp.secs + 10^-9 * e.data.header.stamp.nsecs;
            obj.output_time_history(end+1) = ros_time;
        end
        
        function input_callback(obj, ~, e)
            obj.input_history(end+1,:) = [e.data.vertical_motor e.data.thrust_motor, e.data.servo];
            ros_time = e.data.header.stamp.secs + 10^-9 * e.data.header.stamp.nsecs;
            obj.input_time_history(end+1) = ros_time;
        end
            
        function close(obj)
            obj.ws.close();
        end
        
    end

end