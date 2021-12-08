%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Creator: Mateus Valverde                        %
% Version Name: inner_layers                      %    
% ORCID: https://orcid.org/0000-0001-7900-8061    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef inner_layers
    properties (Access = 'private')
        net = [];
        input_buffer = [];
        label_buffer = [];
        params = [];
    end
    properties (Access = 'public')
    
    end
    
    methods(Access = 'public')
        function obj = inner_layers(params)
            obj.params = params;
            
            obj.net = feedforwardnet(params.hidden_size, params.training_fcn);
            obj.net.layers{1}.transferFcn = 'poslin';
            obj.net.layers{2}.transferFcn = 'poslin';
            obj.net.layers{3}.transferFcn = 'tansig';
            
            obj.net.inputs{1}.size = params.nx;
            obj.net.layers{3}.size = params.n2;
            
            obj.net.initFcn = 'initlay';
            
            obj.net.layers{1}.initFcn = 'initwb';
            obj.net.layers{2}.initFcn = 'initwb';
            obj.net.layers{3}.initFcn = 'initwb';
            
            if params.fix_seed
                rng(0);
            end
            
            obj.net.IW{1} = 1*randn(params.hidden_size(1),obj.net.inputs{1}.size);
            obj.net.b{1} = 1*randn(params.hidden_size(1),1);
            obj.net.LW{2} = 1*randn(params.hidden_size(2),params.hidden_size(1));
            obj.net.b{2} = 1*randn(params.hidden_size(2),1);
            obj.net.LW{3,2} = 1*randn(params.n2,params.hidden_size(2));
            obj.net.b{3} = 1*randn(params.n2,1);
            
            % Disable training pop up window
            obj.net.trainParam.showWindow = 0;
            
            %view(obj.net)
            %pause(100)
        end
        
        function output = forward(obj, x)
            output = [1; obj.net(x)];
        end
        
        function obj = add_label(obj, x, label)
            %size(x)
            
            if ~isempty(obj.label_buffer)
                gamma = norm(label - obj.label_buffer(end))^2/norm(label);
            else
                gamma = obj.params.label_tol + 1;
            end
            
            if gamma >= obj.params.label_tol
                %disp('adding new value to the buffer...');
                % Add input and label to the buffer
                obj.input_buffer = [x, obj.input_buffer];
                obj.label_buffer = [label, obj.label_buffer];
                % Add output to the output buffer
                %obj.output_buffer = [obj.output_buffer, output];
                
                p = length(obj.input_buffer);
                if p > obj.params.p_max                    
                    T = obj.input_buffer;
                    T_label = obj.label_buffer;
                    %T_output = obj.output_buffer;
                    old_eigs = min(sqrt(eig(T_label(:,2:p)*T_label(:,2:p)')));
                    %disp(T);
                    %disp(x);
                    
                    for ii = 2:p
                        T_label(:,ii) = label;%input new point
                        %dets(ii)= det(A*A');%calculate determinant
                        eigs(ii) = min(sqrt(eig(T_label(:,2:p)*T_label(:,2:p)'))); %min(svd(A*A'));%calculate min eigenvalue
                        T_label=obj.label_buffer;%recover old
                    end
                    
                    [Y,I]=max(eigs);
                    
                    if Y>old_eigs
                        T(:,I) = x;
                        T_label(:,I) = label;
                        %T_label(:,I) = output;
                        obj.input_buffer = T(:, 2:end);
                        obj.label_buffer = T_label(:, 2:end);
                        %obj.output_buffer = T_output(:, 2:end);                       
                    end
                    
%                     %Remove last value from buffer
%                     obj.input_buffer = obj.input_buffer(1:end-1);
%                     obj.label_buffer = obj.label_buffer(1:end-1);
%                     obj.output_buffer = obj.output_buffer(1:end-1);
                end
            end
        end
        
        function [obj, acc] = train_layers(obj, K_a)
            % Shuffle buffers
            idx = randperm(size(obj.input_buffer,2));
            n_samples = min(obj.params.samples, size(obj.input_buffer,2));
            input = obj.input_buffer(:,idx(1:n_samples));
            %label = lsqminnorm(K_a', obj.label_buffer(idx(1:n_samples)));
            %label = pinv(K_a')*obj.label_buffer(idx(1:n_samples));
            label = K_a'\obj.label_buffer(:,idx(1:n_samples));
            obj.net = train(obj.net, input, label(2:end,:));
            y = obj.net(input);
            acc = perform(obj.net, y, label(2:end,:));
        end
    end
end
