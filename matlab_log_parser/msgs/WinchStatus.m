classdef WinchStatus < handle
    properties
        op_mode
        motor_position
        motor_speed
        motor_torque
        cable_length
        aux_position
    end
    
    methods
        function set(obj, motor_status_packed)
            obj.op_mode = motor_status_packed(1);
            obj.motor_position = motor_status_packed(2);
            obj.motor_speed = motor_status_packed(3);
            obj.motor_torque = motor_status_packed(4);
            obj.cable_length = motor_status_packed(5);
            obj.aux_position = motor_status_packed(6);
        end
        
        function append(obj, motor_status_packed)
            obj.op_mode(end + 1) = motor_status_packed(1);
            obj.motor_position(end + 1) = motor_status_packed(2);
            obj.motor_speed(end + 1) = motor_status_packed(3);
            obj.motor_torque(end + 1) = motor_status_packed(4);
            obj.cable_length(end + 1) = motor_status_packed(5);
            obj.aux_position(end + 1) = motor_status_packed(6);
        end        
        
        function clear(obj)
            obj.op_mode = [];
            obj.motor_position = [];
            obj.motor_speed = [];
            obj.motor_torque = [];
            obj.cable_length = [];
            obj.aux_position = [];
        end
    end
end
